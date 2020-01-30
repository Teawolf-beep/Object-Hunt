//
// Created by robert on 08.01.20.
//

// System includes
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <chrono>
#include <unistd.h>
#include <poll.h>
#include <algorithm>

// Project includes
#include "ConnectionManager.h"
#include "ConnectionManagerException.h"
//#include "ObjectHuntDefinitions.h"

#include "../../object_hunt_base/shared/include/ObjectHuntDefinitions.h"

ConnectionManager::ConnectionManager()
{
    if (pipe(this->control_pipe))
        throw ConnectionManagerException("ConnectionManager() -> Error while creating the control pipe: " +
                                  std::string(std::strerror(errno)));
}

ConnectionManager::~ConnectionManager()
{
    printf("~ConnectionManager() -> Collecting active threads...\n");
    bool exit_condition = true;

    if (write(this->control_pipe[1], &exit_condition, sizeof(exit_condition)) == -1)
    {
        fprintf(stderr,"~ConnectionManager() -> "
                       "Error while write() to the control pipe: %s\n", strerror(errno));
    }
    this->connection_handler.join();

    printf("~ConnectionManager() -> Closing active connections...\n");
    this->removeAllConnections();
}

void ConnectionManager::removeConnection(int fd)
{
    ConnectionManager::closeSocket(fd);
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    this->active_connections.erase(std::remove(this->active_connections.begin(),
            this->active_connections.end(), fd), this->active_connections.end());
    printf("ConnectionManager::removeConnection() -> Connection on fd %d removed from the active connections\n", fd);
}

void ConnectionManager::removeAllConnections()
{
    for (int fd : this->active_connections)
    {
        ConnectionManager::closeSocket(fd);
        this->active_connections.erase(std::remove(this->active_connections.begin(),
                this->active_connections.end(), fd), this->active_connections.end());
    }
}

const std::vector<int> & ConnectionManager::getConnections()
{
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    return this->active_connections;
}

void ConnectionManager::addConnection(int fd)
{
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    this->active_connections.push_back(fd);
    printf("ConnectionManager::addConnection() -> Connection on fd %d added to the active connections\n", fd);
}

void ConnectionManager::setNavigationControlPipe(int fd)
{
    if (fd < 0)
    {
        printf("ConnectionManager::setNavigationControlPipe() -> Invalid file descriptor (%d) received!\n", fd);
        return;
    }
    this->navigation_control_pipe = fd;

    int listener_socket = ConnectionManager::openSocket(Socket::Ports::NAVIGATION);
    this->connection_handler = std::thread(&ConnectionManager::handleConnections, this, listener_socket);
}

int ConnectionManager::connectToBase(uint16_t base_port)
{
    int ret_fd;
    struct sockaddr_in serv_addr = {AF_INET, htons(base_port)};

    ret_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ret_fd < 0) fprintf(stderr, "ConnectionManager::connectToBase() -> "
                                    "Error while socket creation: %s\n", strerror(errno));
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <=0)
    {
        fprintf(stderr, "ConnectionManager::connectToBase() -> "
                        "Error while network address conversion: %s\n", strerror(errno));
        return -1;
    }
    while (connect(ret_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
        std::this_thread::sleep_for(std::chrono::seconds(1));

    return ret_fd;
}

int ConnectionManager::openSocket(uint16_t port)
{
    struct sockaddr_in serv_addr;
    const int optval = 1;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while socket creation on port %d: %s\n", port, strerror(errno));
        return -1;
    }
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int)))
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while setting options on socket: %s\n", strerror(errno));
        return -2;
    }
//    fcntl(fd, F_SETFL, O_NONBLOCK);
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while bind() on socket: %s\n", strerror(errno));
        return -3;
    }
    if (listen(fd, Socket::Settings::QUEUE_LENGTH) < 0)
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while listen() on socket: %s\n", strerror(errno));
        return -4;
    }

    printf("ConnectionManager::openSocket: Opened socket on port %d\n", port);
    return fd;
}

int ConnectionManager::closeSocket(int fd)
{
    if (shutdown(fd, SHUT_RDWR) < 0)
        if (errno != ENOTCONN && errno != EINVAL)
            printf("Error while closing socket with fd %d\n", fd);
    return close(fd);
}

void ConnectionManager::handleConnections(int server_fd)
{
    bool exit_condition = false;
    struct pollfd pfd[2];

    pfd[0].fd = control_pipe[0];
    pfd[0].events = POLLIN;
    pfd[1].fd = server_fd;
    pfd[1].events = POLLIN;

    while (!exit_condition)
    {
        if (poll(pfd, 2, -1) == -1)
        {
            fprintf(stderr, "ConnectionManager::handleConnections() -> "
                            "Error while poll(): %s\n", strerror(errno));
            return;
        }
        if (pfd[0].revents & POLLIN)
        {
            ssize_t n = read(pfd[0].fd, &exit_condition, sizeof(exit_condition));
            if (n == -1)
            {
                fprintf(stderr, "ConnectionManager::handleConnections() -> "
                                "Error while read(): %s\n", strerror(errno));
                return;
            }
            continue;
        }
        else if (pfd[1].revents & POLLIN)
        {
            struct sockaddr_in cli_addr;
            socklen_t cli_addr_size = sizeof(cli_addr);

            int socket_fd = accept(server_fd, (struct sockaddr *) &cli_addr, &cli_addr_size);
            if(socket_fd < 0)
            {
                fprintf(stderr,"ConnectionManager::handleConnections() -> "
                               "Error while accepting connection on socket with file descriptor %d: %s\n", socket_fd, strerror(errno));
                return;
            }
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &cli_addr.sin_addr, ip, sizeof (ip));
            printf("ConnectionManager::handleConnections() -> "
                   "Established a connection to %s on port %d\n", ip, htons (cli_addr.sin_port));
            this->addConnection(socket_fd);

            if (write(this->navigation_control_pipe, &exit_condition, sizeof(exit_condition)) == -1)
            {
                fprintf(stderr,"ConnectionManager::handleConnections() -> "
                               "Error while write() to control pipe: %s\n", strerror(errno));
                return;
            }
            if (listen(server_fd, Socket::Settings::QUEUE_LENGTH) < 0)
            {
                fprintf(stderr, "ObjectHunter::handleConnections() -> "
                                "Error while listening on socket: %s\n", strerror(errno));
                return;
            }
        }
    }
}
