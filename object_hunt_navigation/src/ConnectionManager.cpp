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
    // Try to create an internal control pipe. True, if an error occurred
    if (pipe(this->control_pipe))
        throw ConnectionManagerException("ConnectionManager() -> Error while creating the control pipe: " +
                                  std::string(std::strerror(errno)));
}

ConnectionManager::~ConnectionManager()
{
    printf("~ConnectionManager() -> Collecting active threads...\n");
    // Create an exit condition variable with value true
    bool exit_condition = true;

    // // Write the exit condition variable to the internal control pipe
    if (write(this->control_pipe[1], &exit_condition, sizeof(exit_condition)) == -1)
    {
        fprintf(stderr,"~ConnectionManager() -> "
                       "Error while write() to the control pipe: %s\n", strerror(errno));
    }
    // Wait for the connection thread to terminate and join it
    this->connection_handler.join();

    printf("~ConnectionManager() -> Closing active connections...\n");
    // Close all existing connections
    this->removeAllConnections();
}

void ConnectionManager::removeConnection(int fd)
{
    // Close the socket associated with the passed file descriptor
    ConnectionManager::closeSocket(fd);
    // Lock the connection mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    // Us the erase remove idiom to remove the passed file descriptor
    this->active_connections.erase(std::remove(this->active_connections.begin(),
            this->active_connections.end(), fd), this->active_connections.end());
    printf("ConnectionManager::removeConnection() -> Connection on fd %d removed from the active connections\n", fd);
}

void ConnectionManager::removeAllConnections()
{
    // Remove all existing connections in a loop
    for (int fd : this->active_connections)
    {
        // Close the socket associated with the respecting file descriptor
        ConnectionManager::closeSocket(fd);
        // Us the erase remove idiom to remove the respecting file descriptor
        this->active_connections.erase(std::remove(this->active_connections.begin(),
                this->active_connections.end(), fd), this->active_connections.end());
    }
}

const std::vector<int> & ConnectionManager::getConnections()
{
    // Lock the connection mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    // Return a reference to the existing connections vector
    return this->active_connections;
}

void ConnectionManager::addConnection(int fd)
{
    // Lock the connection mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    // Add the passed file descriptor to the vector of existing connections
    this->active_connections.push_back(fd);
    printf("ConnectionManager::addConnection() -> Connection on fd %d added to the active connections\n", fd);
}

void ConnectionManager::setNavigationControlPipe(int fd)
{
    // Check the value of the passed file descriptor
    if (fd < 0)
    {
        printf("ConnectionManager::setNavigationControlPipe() -> Invalid file descriptor (%d) received!\n", fd);
        return;
    }
    // Store the passed file descriptor
    this->navigation_control_pipe = fd;
    // Open a listening socket for further process
    int listener_socket = ConnectionManager::openSocket(Socket::Ports::NAVIGATION);
    // Start a thread with the task to listen for incoming connections
    this->connection_handler = std::thread(&ConnectionManager::handleConnections, this, listener_socket);
}

int ConnectionManager::connectToBase(uint16_t base_port)
{
    int ret_fd;
    struct sockaddr_in serv_addr = {AF_INET, htons(base_port)};

    // Create a socket
    ret_fd = socket(AF_INET, SOCK_STREAM, 0);
    // True, if an error occurred while socket creation
    if (ret_fd < 0) fprintf(stderr, "ConnectionManager::connectToBase() -> "
                                    "Error while socket creation: %s\n", strerror(errno));
    // Try to convert the IP address of the local host
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <=0)
    {
        fprintf(stderr, "ConnectionManager::connectToBase() -> "
                        "Error while network address conversion: %s\n", strerror(errno));
        return -1;
    }
    // Try to connect to the base process in a loop
    while (connect(ret_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
        std::this_thread::sleep_for(std::chrono::seconds(1));
    // Return the file descriptor of teh file descriptor
    return ret_fd;
}

int ConnectionManager::openSocket(uint16_t port)
{
    struct sockaddr_in serv_addr;
    const int optval = 1;

    // Create a socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    // True, if an error occurred while socket creation
    if (fd < 0)
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while socket creation on port %d: %s\n", port, strerror(errno));
        return -1;
    }
    // Set the socket options
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int)))
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while setting options on socket: %s\n", strerror(errno));
        return -2;
    }
    // Set the sockaddr_in struct to zero
    memset(&serv_addr, 0, sizeof(serv_addr));
    // Specify options for the socket
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    // Set the port for the socket
    serv_addr.sin_port = htons(port);
    // Bind the socket
    if (bind(fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        fprintf(stderr, "ConnectionManager::openSocket() -> "
                        "Error while bind() on socket: %s\n", strerror(errno));
        return -3;
    }
    // Listen on the socket to bring it in a valid state for accept()
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
    // Try to shut down the socket properly
    if (shutdown(fd, SHUT_RDWR) < 0)
        if (errno != ENOTCONN && errno != EINVAL)
            printf("Error while closing socket with fd %d\n", fd);
    // Return teh result of the call to close()
    return close(fd);
}

void ConnectionManager::handleConnections(int server_fd)
{
    bool exit_condition = false;
    struct pollfd pfd[2];

    // Initialize the poll file descriptors with the file descriptors and desired events
    pfd[0].fd = control_pipe[0];
    pfd[0].events = POLLIN;
    pfd[1].fd = server_fd;
    pfd[1].events = POLLIN;
    // Handle connections until the exit condition is true
    while (!exit_condition)
    {
        // Wait for events without time limit
        if (poll(pfd, 2, -1) == -1)
        {
            fprintf(stderr, "ConnectionManager::handleConnections() -> "
                            "Error while poll(): %s\n", strerror(errno));
            return;
        }
        // True, if a message was written to the internal control pipe
        if (pfd[0].revents & POLLIN)
        {
            //Update the exit condition variable
            ssize_t n = read(pfd[0].fd, &exit_condition, sizeof(exit_condition));
            if (n == -1)
            {
                fprintf(stderr, "ConnectionManager::handleConnections() -> "
                                "Error while read(): %s\n", strerror(errno));
                return;
            }
            continue;
        }
        // True, if a process tries to establish a connection with this process
        else if (pfd[1].revents & POLLIN)
        {
            struct sockaddr_in cli_addr;
            socklen_t cli_addr_size = sizeof(cli_addr);

            // Accept the connection
            int socket_fd = accept(server_fd, (struct sockaddr *) &cli_addr, &cli_addr_size);
            // True, if an error occurred
            if(socket_fd < 0)
            {
                fprintf(stderr,"ConnectionManager::handleConnections() -> "
                               "Error while accepting connection on socket with file descriptor %d: %s\n", socket_fd, strerror(errno));
                return;
            }
            // Transform the IP address to human readable format
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &cli_addr.sin_addr, ip, sizeof (ip));
            printf("ConnectionManager::handleConnections() -> "
                   "Established a connection to %s on port %d\n", ip, htons (cli_addr.sin_port));
            // Add the established to the vector of existing connections
            this->addConnection(socket_fd);
            // Write the exit condition variable to the navigation pipe. This will notify the parent
            // thread about the new connection.
            if (write(this->navigation_control_pipe, &exit_condition, sizeof(exit_condition)) == -1)
            {
                fprintf(stderr,"ConnectionManager::handleConnections() -> "
                               "Error while write() to control pipe: %s\n", strerror(errno));
                return;
            }
            // Listen on the socket to bring it in a valid state for further calls to accept()
            if (listen(server_fd, Socket::Settings::QUEUE_LENGTH) < 0)
            {
                fprintf(stderr, "ObjectHunter::handleConnections() -> "
                                "Error while listening on socket: %s\n", strerror(errno));
                return;
            }
        }
    }
}
