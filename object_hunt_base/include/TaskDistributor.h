//
// Created by robert on 21.11.19.
//

#ifndef OBJECT_HUNT_BASE_TASKDISTRIBUTOR_H
#define OBJECT_HUNT_BASE_TASKDISTRIBUTOR_H

// System includes
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>

// Project includes
#include "TaskDistributorException.h"

/**
 * Provides definitions for the state of the TaskDistributor class.
 */
namespace StateMachine
{
    /**
     * The possible states of the TaskDistributor class.
     */
    enum States
    {
        INITIALIZATION, /**< Initialization state. The process is initializing itself.*/
        CONNECTING, /**< Waiting for connection. The process waits for a connection with the navigation process.*/
        CONNECTED, /**< Connection established. A connection to the navigation process is established.*/
        RUNNING /**< Process running. The process is running as expected and ready to receive commands and requests.*/
    };
}
typedef StateMachine::States States;

// Forward declarations
class GPIO;
class HardwareInterface;

/**
 * The application class of this process.
 * This class will be called by the main() function and handles the further program execution.
 */
class TaskDistributor
{
public:

    /**
     * Custom constructor.
     * Initializes needed resources.
     *
     * @param sfd A signal file descriptor to catch the signals SIGINT and SIGTERM directly while poll.
     * Needs to be set up in the main() function. Signal file descriptor are exclusive linux only.
     * @throws TaskDistributorException in case an error occurred while initialization.
     */
    TaskDistributor(int sfd);

    /**
     * Custom destructor.
     * Frees resources and collects active threads.
     */
    ~TaskDistributor();

    /**
     * The application method.
     * From here the program starts running. Needs to be called after class initialization from the main() function.
     *
     * @return A signed integer number which indicates the termination status of the application.
     * Please see the namespace ReturnValue for possible values.
     */
    int8_t run();

private:
    static int openSocket(uint16_t port);
    static int closeSocket(int fd);
    static void printReadError(const std::string& part);
    static void printIncompleteError(const std::string& part);

    int waitForConnection(int server_fd);
    int8_t receiveCommands();
    void processIncomingMessage(uint8_t action);
    void terminateRotationAgent();
    void startNavigation();
    void terminateNavigation();

    std::shared_ptr<GPIO> gpio;
    std::shared_ptr<HardwareInterface> hw_interface;
    int signal_fd;
    int start_stop_fd;
    int navigation_fd;
    States state;
    std::thread rotation_agent;
};

#endif //OBJECT_HUNT_BASE_TASKDISTRIBUTOR_H
