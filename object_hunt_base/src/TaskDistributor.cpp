//
// Created by robert on 27.11.19.
//

// System includes
#include <memory>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <string>
#include <netinet/in.h>
#include <iostream>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/signalfd.h>
#include <csignal>

// Project includes
#include "TaskDistributor.h"
#include "ObjectHuntDefinitions.h"
#include "GPIO.h"
#include "HardwareInterface.h"

TaskDistributor::TaskDistributor(int sfd)
        : gpio(std::make_shared<GPIO>(GPIO_USER_FILENAME)),
          hw_interface(std::make_shared<HardwareInterface>(gpio)),
          signal_fd(sfd),
          start_stop_fd(gpio->setSingleKernelDriver(Button::START_STOP, PullState::OFF, Edge::FALLING)),
          state(StateMachine::INITIALIZATION)
{
    if (this->start_stop_fd < 0)
        throw TaskDistributorException("TaskDistributor() -> Error while setting up the power off button!");
}

TaskDistributor::~TaskDistributor()
{
    printf("Shutting down. Active threads will be collected...\n");
//    while (this->hw_interface->ultraSonicRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
//    printf("~TaskDistributor() GPIO count: %ld\n", this->gpio.use_count());
    this->terminateRotationAgent();
}

int8_t TaskDistributor::run()
{
    int8_t ret = 1;
    int result;

    int server_fd = TaskDistributor::openSocket(Socket::Ports::BASE);
    this->state = StateMachine::CONNECTING;
    while (ret > 0)
    {
        result = this->waitForConnection(server_fd);
        if (result)
        {
            printf("TaskDistributor::run() -> "
                   "Could not establish a connection to the navigation process! (return value: %d) \n", result);
            if (result == ReturnValue::POWER_OFF) return result;
            return ReturnValue::NO_CONNECTION;
        }
        else ret = this->receiveCommands();
        TaskDistributor::closeSocket(this->navigation_fd);
    }
    TaskDistributor::closeSocket(server_fd);
    return ret;
}

int8_t TaskDistributor::receiveCommands()
{
    int8_t ret = ReturnValue::LEAVE_SUCCESS;
    struct pollfd pfd[3];
    struct signalfd_siginfo signal_info;
    int ret_poll;
    ssize_t n;
    uint8_t action;

    pfd[0].fd = this->start_stop_fd;
    pfd[0].events = POLLPRI;
    pfd[1].fd = this->signal_fd;
    pfd[1].events = POLLIN;
    pfd[2].fd = this->navigation_fd;
    pfd[2].events = POLLIN;

    while (true)
    {
        // Wait for events without time limit
        ret_poll = poll(pfd, 3, -1);
        if (ret_poll == -1)
        {
            fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                            "Error while poll(): %s...However, returning to work\n", strerror(errno));
        }
        if (pfd[0].revents & POLLPRI)
        {
            char buffer[8];

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            lseek(this->start_stop_fd, 0, SEEK_SET);
            read(this->start_stop_fd, &buffer, sizeof(buffer));
            if (this->state == StateMachine::CONNECTED) this->startNavigation();
            else
            {
                this->terminateNavigation();
                break;
            }
        }
        else if (pfd[1].revents & POLLIN)
        {
            n = read(pfd[1].fd, &signal_info, sizeof(signal_info));
            if (n == -1)
            {
                fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                                "Error while read() on signal pipe: %s\n", strerror(errno));
            }
            if ((signal_info.ssi_signo == SIGTERM) || (signal_info.ssi_signo == SIGINT))
            {
                this->terminateNavigation();
                printf("TaskDistributor::receiveCommands() -> Signal received\n");
                break;
            }
        }
        else if (pfd[2].revents & POLLIN)
        {
            n = read(pfd[2].fd, &action, sizeof(action));
            if (n == -1)
            {
                fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                                "Error while read() on navigation fd: %s...However, returning to work\n", strerror(errno));
                continue;
            }
            else if (n == 0)
            {
                printf("TaskDistributor::receiveCommands() -> Lost connection..."
                       "Returning to listen on port %d\n", Socket::Ports::BASE);
                this->hw_interface->stopMotors();
                this->state = StateMachine::CONNECTING;
                ret = 1;
                break;
            }
            else processIncomingMessage(action);
        }
    }
    return ret;
}

void TaskDistributor::processIncomingMessage(uint8_t action)
{
    static ssize_t n;

    switch (action)
    {
        case Action::MOTOR:
        {
            struct Motor::Command command;
            n = read(this->navigation_fd, &command, sizeof(command));
            if (n == -1) TaskDistributor::printReadError("motor");
            else if (n != sizeof(command)) TaskDistributor::printIncompleteError("motor");
            else this->hw_interface->moveMotor(&command);
            break;
        }
        case Action::STEPPER:
        {
            struct Stepper::Command command;
            n = read(this->navigation_fd, &command, sizeof(command));
            if (n == -1) TaskDistributor::printReadError("stepper");
            else if (n != sizeof(command)) TaskDistributor::printIncompleteError("stepper");
            else this->hw_interface->moveStepper(&command);
            break;
        }
        case Action::ULTRA_SONIC:
        {
            struct UltraSonic::Request request;
            n = read(this->navigation_fd, &request, sizeof(request));
            if (n == -1) TaskDistributor::printReadError("ultra sonic");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("ultra sonic");
            else this->hw_interface->startUltraSonic(this->navigation_fd, &request);
            break;
        }
        case Action::REVOLUTION:
        {
            struct Revolution::Request request;
            n = read(this->navigation_fd, &request, sizeof(request));
            if (n == -1) TaskDistributor::printReadError("revolution");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("revolution");
            else this->hw_interface->getRevolution(this->navigation_fd, &request);
            break;
        }
        case Action::SPEED:
        {
            struct Speed::Request request;
            n = read(this->navigation_fd, &request, sizeof(request));
            if (n == -1) TaskDistributor::printReadError("value");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("value");
            else this->hw_interface->getSpeed(this->navigation_fd, request.id);
            break;
        }
        case Action::ROTATION:
        {
            struct Rotation::Command request;
            n = read(this->navigation_fd, &request, sizeof(request));
            if (n == -1) TaskDistributor::printReadError("rotation");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("rotation");
            else
            {
                this->terminateRotationAgent();
                this->rotation_agent = std::thread(&HardwareInterface::startRotation,
                                                   this->hw_interface, this->navigation_fd, &request);
            }
            break;
        }
        case Action::ORIENTATION:
        {
            struct Orientation::Request request;
            n = read(this->navigation_fd, &request, sizeof(request));
            if (n == -1) TaskDistributor::printReadError("orientation");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("orientation");
            else this->hw_interface->getOrientation(this->navigation_fd, request.id);
            break;
        }
        default:
            printf("TaskDistributor::processIncomingMessage() -> "
                   "Unknown action received: %d...However, returning to work\n", action);
            break;
    }
}

int TaskDistributor::waitForConnection(int server_fd)
{
    struct signalfd_siginfo signal_info;
    struct pollfd pfd[3];

    pfd[0].fd = this->start_stop_fd;
    pfd[0].events = POLLPRI;
    pfd[1].fd = this->signal_fd;
    pfd[1].events = POLLIN;
    pfd[2].fd = server_fd;
    pfd[2].events = POLLIN;

    if (poll(pfd, 3, -1) == -1)
    {
        fprintf(stderr, "TaskDistributor::waitForConnection() -> "
                        "Error while poll(): %s\n", strerror(errno));
        return -2;
    }
    if ((pfd[0].revents & POLLPRI))
    {
        char buffer[8];

        lseek(this->start_stop_fd, 0, SEEK_SET);
        read(this->start_stop_fd, &buffer, sizeof(buffer));
        return ReturnValue::POWER_OFF;
    }
    if (pfd[1].revents & POLLIN)
    {
        ssize_t n = read(pfd[1].fd, &signal_info, sizeof(signal_info));
        if (n == -1)
        {
            fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                            "Error while read() on signal pipe: %s\n", strerror(errno));
        }
        if ((signal_info.ssi_signo == SIGTERM) || (signal_info.ssi_signo == SIGINT))
        {
            printf("TaskDistributor::waitForConnection() -> Termination signal received\n");
            return ReturnValue::NO_CONNECTION;
        }
    }
    if (pfd[2].revents & POLLIN)
    {
        struct sockaddr_in cli_addr;
        socklen_t cli_addr_size = sizeof(cli_addr);
        char ip[INET_ADDRSTRLEN];

        this->navigation_fd = accept(server_fd, (struct sockaddr *) &cli_addr, &cli_addr_size);
        if(this->navigation_fd < 0)
        {
            fprintf(stderr,"TaskDistributor::waitForConnection() -> "
                           "Error while accepting connection on file descriptor %d: %s\n",
                    this->navigation_fd, strerror(errno));
            return -4;
        }
        inet_ntop(AF_INET, &cli_addr.sin_addr, ip, sizeof(ip));
        this->state = StateMachine::CONNECTED;
        printf("TaskDistributor::waitForConnection() -> "
               "Established a connection to %s on port %d\n", ip, htons(cli_addr.sin_port));
    }
    return 0;
}

void TaskDistributor::startNavigation()
{
    std::this_thread::sleep_for(std::chrono::milliseconds (500));
    if (write(this->navigation_fd, &Action::START, sizeof(Action::START)) == -1)
    {
        fprintf(stderr, "TaskDistributor::startNavigation() -> "
                        "Error while write() start signal: %s\n", strerror(errno));
    }
    else
    {
        this->state = StateMachine::RUNNING;
        printf("TaskDistributor::startNavigation() -> Navigation started\n");
    }
}

void TaskDistributor::terminateNavigation()
{
    if (write(this->navigation_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
    {
        fprintf(stderr, "TaskDistributor::terminateNavigation() -> "
                        "Error while write() terminate signal: %s\n", strerror(errno));
    }
    else printf("TaskDistributor::terminateNavigation() -> Navigation terminated\n");
    this->hw_interface->stopMotors();
}

int TaskDistributor::openSocket(uint16_t port)
{
    struct sockaddr_in serv_addr;
    const int optval = 1;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while socket creation on port %d: %s\n", port, strerror(errno));
        return -1;
    }
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int)))
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while setting options on socket: %s\n", strerror(errno));
        return -2;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while bind() on socket: %s\n", strerror(errno));
        return -3;
    }

    if (listen(fd, Socket::Settings::QUEUE_LENGTH) < 0)
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while listen() on socket: %s\n", strerror(errno));
        return -4;
    }
    printf("TaskDistributor::openSocket(): Opened socket on port %d\n", port);
    return fd;
}

void TaskDistributor::printReadError(const std::string& part)
{
    fprintf(stderr, "TaskDistributor::processIncomingMessage() -> "
                    "Error while reading %s command: %s...returning to work.\n",
            part.c_str(), strerror(errno));
}

void TaskDistributor::printIncompleteError(const std::string& part)
{
    printf("TaskDistributor::processIncomingMessage() -> "
           "Received incomplete %s command...returning to work.\n", part.c_str());
}

int TaskDistributor::closeSocket(int fd)
{
        if (shutdown(fd, SHUT_RDWR) < 0)
            if (errno != ENOTCONN && errno != EINVAL)
                printf("Error while closing socket with fd %d\n", fd);
        return close(fd);
}

void TaskDistributor::terminateRotationAgent()
{
    if (this->rotation_agent.joinable() && this->hw_interface->rotationMovementRunning())
    {
        bool exit_condition = true;
        write(this->hw_interface->getRotationPipe(), &exit_condition, sizeof(bool));
        this->rotation_agent.join();
    }
    else if (this->rotation_agent.joinable()) this->rotation_agent.join();
}
