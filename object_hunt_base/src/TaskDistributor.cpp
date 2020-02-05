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

// Constructor. Initializes members via member initializer list.
TaskDistributor::TaskDistributor(int sfd)
        : gpio(std::make_shared<GPIO>(GPIO_USER_FILENAME)),
          hw_interface(std::make_shared<HardwareInterface>(gpio)),
          signal_fd(sfd),
          start_stop_fd(gpio->setSingleKernelDriver(Button::START_STOP, PullState::OFF, Edge::FALLING)),
          state(StateMachine::INITIALIZATION)
{
    // True, if an error occurred while assigning the single kernel driver in the member initializer list
    if (this->start_stop_fd < 0)
        throw TaskDistributorException("TaskDistributor() -> Error while setting up the power off button!");
}

TaskDistributor::~TaskDistributor()
{
    printf("Shutting down. Active threads will be collected...\n");
    // Stop the rotation thread
    this->terminateRotationAgent();
}

int8_t TaskDistributor::run()
{
    int8_t ret = 1;
    int result;

    // Open a TCP socket on the passed port
    int server_fd = TaskDistributor::openSocket(Socket::Ports::BASE);
    // Change the active state
    this->state = StateMachine::CONNECTING;
    // Execute the program until termination is requested
    while (ret > 0)
    {
        // Wait for connection from the navigation process
        result = this->waitForConnection(server_fd);
        // True, if no connection could be established
        if (result)
        {
            printf("TaskDistributor::run() -> "
                   "Could not establish a connection to the navigation process! (return value: %d) \n", result);
            if (result == ReturnValue::POWER_OFF) return result;
            return ReturnValue::NO_CONNECTION;
        }
        // Wait for incoming commands, requests and other events
        else ret = this->receiveCommands();
        // Close the socket associated with the last connection
        TaskDistributor::closeSocket(this->navigation_fd);
    }
    // Close the listening socket
    TaskDistributor::closeSocket(server_fd);
    return ret;
}

int8_t TaskDistributor::receiveCommands()
{
    // Initialize variables
    int8_t ret = ReturnValue::LEAVE_SUCCESS;
    struct pollfd pfd[3];
    struct signalfd_siginfo signal_info;
    int ret_poll;
    ssize_t n;
    uint8_t action;

    // Initialize the poll file descriptors with the file descriptors and desired events
    pfd[0].fd = this->start_stop_fd;
    pfd[0].events = POLLPRI;
    pfd[1].fd = this->signal_fd;
    pfd[1].events = POLLIN;
    pfd[2].fd = this->navigation_fd;
    pfd[2].events = POLLIN;
    // Endless loop
    while (true)
    {
        // Wait for events without time limit
        ret_poll = poll(pfd, 3, -1);
        // true, if an error occurred
        if (ret_poll == -1)
        {
            fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                            "Error while poll(): %s...However, returning to work\n", strerror(errno));
        }
        // True, if a signal from the push button was recognized
        if (pfd[0].revents & POLLPRI)
        {
            char buffer[8];
            // Sleep some time to prevent false signals caused by polling
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // Consume the interrupt
            lseek(this->start_stop_fd, 0, SEEK_SET);
            read(this->start_stop_fd, &buffer, sizeof(buffer));
            // Start the navigation process, if a connection is established
            if (this->state == StateMachine::CONNECTED) this->startNavigation();
            // Stop navigation otherwise
            else
            {
                this->terminateNavigation();
                break;
            }
        }
        // True, if a signal from the operating system was sent to this process
        else if (pfd[1].revents & POLLIN)
        {
            // Get the signal
            n = read(pfd[1].fd, &signal_info, sizeof(signal_info));
            // true, if an error occurred while getting the signal
            if (n == -1)
            {
                fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                                "Error while read() on signal pipe: %s\n", strerror(errno));
            }
            // Check, if we are really interested in the caught signal
            if ((signal_info.ssi_signo == SIGTERM) || (signal_info.ssi_signo == SIGINT))
            {
                // Send a stop signal to the navigation process
                this->terminateNavigation();
                printf("TaskDistributor::receiveCommands() -> Signal received\n");
                // Leave the loop
                break;
            }
        }
        // True, if a message from the navigation process was received
        else if (pfd[2].revents & POLLIN)
        {
            // Get the action identifier (first byte only)
            n = read(pfd[2].fd, &action, sizeof(action));
            // True, if an error occurred while getting the action identifier
            if (n == -1)
            {
                fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                                "Error while read() on navigation fd: %s...However, returning to work\n", strerror(errno));
                continue;
            }
            // True, if the connection is closed by the navigation process
            else if (n == 0)
            {
                printf("TaskDistributor::receiveCommands() -> Lost connection..."
                       "Returning to listen on port %d\n", Socket::Ports::BASE);
                // Stop all motors
                this->hw_interface->stopMotors();
                // Change the current state
                this->state = StateMachine::CONNECTING;
                ret = 1;
                break;
            }
            // Get the real message and process it
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
        // The message is a motor command
        case Action::MOTOR:
        {
            struct Motor::Command command;
            // Get the actual command
            n = read(this->navigation_fd, &command, sizeof(command));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("motor");
            else if (n != sizeof(command)) TaskDistributor::printIncompleteError("motor");
            // Pass the command to the hardware interface
            else this->hw_interface->moveMotor(&command);
            break;
        }
        // The message is a stepper command
        case Action::STEPPER:
        {
            struct Stepper::Command command;
            // Get the actual command
            n = read(this->navigation_fd, &command, sizeof(command));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("stepper");
            else if (n != sizeof(command)) TaskDistributor::printIncompleteError("stepper");
            // Pass the command to the hardware interface
            else this->hw_interface->moveStepper(&command);
            break;
        }
        // The message is a ultra sonic request
        case Action::ULTRA_SONIC:
        {
            struct UltraSonic::Request request;
            // Get the actual request
            n = read(this->navigation_fd, &request, sizeof(request));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("ultra sonic");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("ultra sonic");
            // Pass the command to the hardware interface
            else this->hw_interface->startUltraSonic(this->navigation_fd, &request);
            break;
        }
        // The message is a revolution request
        case Action::REVOLUTION:
        {
            struct Revolution::Request request;
            // Get the actual request
            n = read(this->navigation_fd, &request, sizeof(request));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("revolution");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("revolution");
            // Pass the command to the hardware interface
            else this->hw_interface->getRevolution(this->navigation_fd, &request);
            break;
        }
        // The message is a speed request
        case Action::SPEED:
        {
            struct Speed::Request request;
            // Get the actual request
            n = read(this->navigation_fd, &request, sizeof(request));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("value");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("value");
            // Pass the command to the hardware interface
            else this->hw_interface->getSpeed(this->navigation_fd, request.id);
            break;
        }
        // The message is a rotation request
        case Action::ROTATION:
        {
            struct Rotation::Command request;
            // Get the actual request
            n = read(this->navigation_fd, &request, sizeof(request));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("rotation");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("rotation");
            // Trigger a new rotation movement
            else
            {
                // Terminate the running rotation movement (if any is running)
                this->terminateRotationAgent();
                // Start a new rotation movement with the desired parameters
                this->rotation_agent = std::thread(&HardwareInterface::startRotation,
                                                   this->hw_interface, this->navigation_fd, &request);
            }
            break;
        }
        // The message is a orientation request
        case Action::ORIENTATION:
        {
            struct Orientation::Request request;
            // Get the actual request
            n = read(this->navigation_fd, &request, sizeof(request));
            // Check for errors
            if (n == -1) TaskDistributor::printReadError("orientation");
            else if (n != sizeof(request)) TaskDistributor::printIncompleteError("orientation");
            // Pass the command to the hardware interface
            else this->hw_interface->getOrientation(this->navigation_fd, request.id);
            break;
        }
        // Unknown action identifier received
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

    // Initialize the poll file descriptors with the file descriptors and desired events
    pfd[0].fd = this->start_stop_fd;
    pfd[0].events = POLLPRI;
    pfd[1].fd = this->signal_fd;
    pfd[1].events = POLLIN;
    pfd[2].fd = server_fd;
    pfd[2].events = POLLIN;
    // Wait on for events without time limit. True, if an error occurred
    if (poll(pfd, 3, -1) == -1)
    {
        fprintf(stderr, "TaskDistributor::waitForConnection() -> "
                        "Error while poll(): %s\n", strerror(errno));
        return -2;
    }
    // True, if a signal from the power button was received
    if ((pfd[0].revents & POLLPRI))
    {
        char buffer[8];
        // Clear interrupt
        lseek(this->start_stop_fd, 0, SEEK_SET);
        read(this->start_stop_fd, &buffer, sizeof(buffer));
        return ReturnValue::POWER_OFF;
    }
    // True, if a sgnal from the operating system was received
    if (pfd[1].revents & POLLIN)
    {
        // Get the signal
        ssize_t n = read(pfd[1].fd, &signal_info, sizeof(signal_info));
        // True, if an error occurred while getting the signal
        if (n == -1)
        {
            fprintf(stderr, "TaskDistributor::receiveCommands() -> "
                            "Error while read() on signal pipe: %s\n", strerror(errno));
        }
        // Check, if we the signal really is for us (should always be true)
        if ((signal_info.ssi_signo == SIGTERM) || (signal_info.ssi_signo == SIGINT))
        {
            printf("TaskDistributor::waitForConnection() -> Termination signal received\n");
            return ReturnValue::NO_CONNECTION;
        }
    }
    // True, if a process (hopefully the navigation process) wants to establish a connection
    if (pfd[2].revents & POLLIN)
    {
        struct sockaddr_in cli_addr;
        socklen_t cli_addr_size = sizeof(cli_addr);
        char ip[INET_ADDRSTRLEN];

        // Accept the connection
        this->navigation_fd = accept(server_fd, (struct sockaddr *) &cli_addr, &cli_addr_size);
        // True, if an error occurred
        if(this->navigation_fd < 0)
        {
            fprintf(stderr,"TaskDistributor::waitForConnection() -> "
                           "Error while accepting connection on file descriptor %d: %s\n",
                    this->navigation_fd, strerror(errno));
            return -4;
        }
        // Transform the IP address to human readable format
        inet_ntop(AF_INET, &cli_addr.sin_addr, ip, sizeof(ip));
        // Change the actual state
        this->state = StateMachine::CONNECTED;
        printf("TaskDistributor::waitForConnection() -> "
               "Established a connection to %s on port %d\n", ip, htons(cli_addr.sin_port));
    }
    return 0;
}

void TaskDistributor::startNavigation()
{
    std::this_thread::sleep_for(std::chrono::milliseconds (500));
    // Write the start action to the navigation process. True, if an error occurred
    if (write(this->navigation_fd, &Action::START, sizeof(Action::START)) == -1)
    {
        fprintf(stderr, "TaskDistributor::startNavigation() -> "
                        "Error while write() start signal: %s\n", strerror(errno));
    }
    // Everything went well, the communication should start in short time
    else
    {
        this->state = StateMachine::RUNNING;
        printf("TaskDistributor::startNavigation() -> Navigation started\n");
    }
}

void TaskDistributor::terminateNavigation()
{
    // Send the termination action to the navigation signal. True, if an error occurred
    if (write(this->navigation_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
    {
        fprintf(stderr, "TaskDistributor::terminateNavigation() -> "
                        "Error while write() terminate signal: %s\n", strerror(errno));
    }
    else printf("TaskDistributor::terminateNavigation() -> Navigation terminated\n");
    // Stop all motors
    this->hw_interface->stopMotors();
}

int TaskDistributor::openSocket(uint16_t port)
{
    struct sockaddr_in serv_addr;
    const int optval = 1;

    // Open a new socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    // True, if an error occurred while opening the socket
    if (fd < 0)
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while socket creation on port %d: %s\n", port, strerror(errno));
        return -1;
    }
    // Set the socket options
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int)))
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while setting options on socket: %s\n", strerror(errno));
        return -2;
    }
    // Make the socket nonblocking (TODO why do we need this? Seems like an artifact from the testing phase
    // and should be removed in the future)
    fcntl(fd, F_SETFL, O_NONBLOCK);
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
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while bind() on socket: %s\n", strerror(errno));
        return -3;
    }
    // Listen on the socket to bring it in a valid state for accept
    if (listen(fd, Socket::Settings::QUEUE_LENGTH) < 0)
    {
        fprintf(stderr, "TaskDistributor::openSocket() -> "
                        "Error while listen() on socket: %s\n", strerror(errno));
        return -4;
    }
    printf("TaskDistributor::openSocket(): Opened socket on port %d\n", port);
    // Return the file descriptor for the opened socket
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
    // Try to shut down the socket properly
    if (shutdown(fd, SHUT_RDWR) < 0)
        if (errno != ENOTCONN && errno != EINVAL)
            printf("Error while closing socket with fd %d\n", fd);
    // Return teh result of the call to close()
    return close(fd);
}

void TaskDistributor::terminateRotationAgent()
{
    // Check, if the rotation thread is actually running
    if (this->rotation_agent.joinable() && this->hw_interface->rotationMovementRunning())
    {
        // Create an exit condition variable with value true
        bool exit_condition = true;
        // Write the exit condition variable to the rotation pipe of the HardwareInterface class
        write(this->hw_interface->getRotationPipe(), &exit_condition, sizeof(bool));
        // Wait for the rotation thread to terminate and join it
        this->rotation_agent.join();
    }
    // Check, if the rotation thread can be joined
    else if (this->rotation_agent.joinable()) this->rotation_agent.join();
}
