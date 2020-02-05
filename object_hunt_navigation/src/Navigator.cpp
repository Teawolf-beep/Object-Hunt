//
// Created by robert on 10.12.19.
//

// System includes
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <poll.h>
#include <cmath>

// Project includes
#include "ObjectHuntDefinitions.h"
#include "Navigator.h"
#include "ConnectionManager.h"

#include "../../object_hunt_base/shared/include/ObjectHuntDefinitions.h"

// Constructor. Initializes members via the member initializer list.
Navigator::Navigator()
        : connection_manager(std::make_unique<ConnectionManager>()),
          base_fd(ConnectionManager::connectToBase(Socket::Ports::BASE)),
          us_pending(false),
          orientation_pending(false),
          speed_pending(false),
          hunt_over(false),
          mapping_fd(-1),
          state(States::STARTING)
{
    // True, if an error occurred while connecting to the base in the member initializer list
    if (base_fd < 0) throw NavigatorException("ConnectionManagerException() -> "
                                                      "Error while connecting to base socket!");
    // Try to create an internal control pipe. True, if an error occurred
    if (pipe(this->control_pipe))
        throw NavigatorException("Navigator() -> Error while creating the main pipe: " +
                                 std::string(std::strerror(errno)));
    this->connection_manager->setNavigationControlPipe(this->control_pipe[1]);
}

Navigator::~Navigator()
{
    printf("~Navigator() -> Closing base socket...\n");
    // Close the connection to the base process
    ConnectionManager::closeSocket(this->base_fd);
}

int8_t Navigator::run()
{
    std::chrono::steady_clock::time_point reference_time = std::chrono::steady_clock::now();
    struct pollfd pfd[Socket::Settings::MAX_CONNECTIONS];
    bool exit_condition = false;
    bool refresh_connections = true;
    int i, processing_time, timeout, connection_count = 1, ret_poll = 0;
    ssize_t n;
    uint8_t action;

    // Initialize the poll file descriptors with the file descriptors and desired events
    pfd[0].fd = this->control_pipe[0];
    pfd[0].events = POLLIN;
    pfd[1].fd = this->base_fd;
    pfd[1].events = POLLIN;
    // Navigate until the exit condition is true
    while (!exit_condition)
    {
        // Check if the connections have to be refreshed
        if (refresh_connections)
        {
            // get a const reference to the vector of active connections
            const std::vector<int> connections = this->connection_manager->getConnections();

            // Iterate over all active connections
            for (connection_count = 2; connection_count < (connections.size()+2); ++connection_count)
            {
                // Check, if the maximal number of active connections is exceeded
                if (connection_count >= Socket::Settings::MAX_CONNECTIONS)
                {
                    printf("Navigator::run() -> The number of established connections exhibits the maximum "
                           "number of connections. Cannot monitor all file descriptor.\nPlease remove some connections.\n");
                    break;
                }
                // Add the file descriptor of the connection to the pollfd array
                pfd[connection_count].fd = connections[connection_count-2];
                // Declare interest on an input event
                pfd[connection_count].events = POLLIN;
            }
            refresh_connections = false;
            printf("Navigator::run() -> Listening on %d file descriptors.\n", connection_count+1);
        }
        // Compute the remaining time to match the loop timeout
        processing_time = int(Navigation::Timeout::GENERAL_MS - std::chrono::duration_cast<std::chrono::milliseconds>
                (std::chrono::steady_clock::now() - reference_time).count());
        // Check, if the computed time is greater than zero. Set it to zero, if not
        timeout = (processing_time > 0) ? processing_time : 0;
        // Wait the computed time for incoming events
        ret_poll = poll(pfd, connection_count, timeout);
        // True, if an error occurred while polling
        if (ret_poll == -1)
        {
            fprintf(stderr, "Navigator::run() -> "
                            "Error while poll(): %s\n", strerror(errno));
        }
        // True, if the timer expired
        else if (ret_poll == 0)
        {
            // Store the current time in a variable
            reference_time = std::chrono::steady_clock::now();
            // check the current state of the car
            if ((this->state == States::MOVING_FORWARD) ||
                (this->state == States::MOVING_BACKWARD))
            {
                // Request sensor readings
                this->requestUltraSonic(0x0F);
                this->requestSpeed();
                this->requestOrientation();
            }
        }
        // True, if a message waits in the control pipe
        else if (pfd[0].revents &  POLLIN)
        {
            // Update the value of the exit condition variable
            n = read(this->control_pipe[0], &exit_condition, sizeof(exit_condition));
            // True, if an error occurred while updating the variable
            if (n == -1)
            {
                fprintf(stderr, "Navigator::run() -> "
                                "Error while read() on control pipe: %s\n", strerror(errno));
                return -1;
            }
            // Check, if teh exit condition is true
            if (exit_condition) printf("Navigator::run() -> Termination signal received\n");
            // Refresh the connections before the next iteration
            else refresh_connections = true;
            continue;
        }
        // True, if we received a message from the base process
        else if (pfd[1].revents & POLLIN)
        {
            // Get the action identifier
            n = read(pfd[1].fd, &action, sizeof(action));
            // True, if an error occurred while getting the action identifier
            if (n == -1)
            {
                fprintf(stderr, "Navigator::run() -> "
                                "Error while read() on base pipe: %s...returning to work.\n", strerror(errno));
                continue;
            }
            // True, if we lost connection to the base process
            else if (n == 0)
            {
                printf("Navigator::run() -> Fatal error! Lost connection to the base process...Aborting!\n");
                return -2;
            }
            // Get the real message and process it
            else this->processIncomingMessage(action);
        }
        // True, if we received a message from another process
        else
        {
            // Iterate over all connections
            for (i = 1; i < connection_count; ++i)
            {
                // Check, if we received a message from the current connection
                if (pfd[i].revents & POLLIN)
                {
                    // Get the action identifier
                    n = read(pfd[i].fd, &action, sizeof(action));
                    // True, if an error occurred while getting the action identifier
                    if (n == -1)
                    {
                        fprintf(stderr, "Navigator::run() -> "
                                        "Error while read(): %s...returning to work.\n", strerror(errno));
                        continue;
                    }
                    // True, if we lost connection to this process
                    else if (n == 0)
                    {
                        // Remove the connection from the active connections
                        connection_manager->removeConnection(pfd[i].fd);
                        // Check, if we lost connection to the mapping process
                        if (pfd[i].fd == this->mapping_fd)
                        {
                            // Reset the mapping file descriptor
                            this->mapping_fd = -1;
                            printf("Navigator::run() -> Connection to mapping process lost\n");
                        }
                        // Refresh the connections before the next iteration
                        refresh_connections = true;
                        // Write the exit condition to this control pipe (TODO why are we doing this? Seems either way
                        // unnecessary or is a bug! Should without doubts be removed in the future!!)
                        write(this->control_pipe[1], &exit_condition, sizeof(exit_condition));
                    }
                    // Get the real message and process it
                    else this->processIncomingMessage(pfd[i].fd, action);
                }
            }
        }
    }
    return 0;
}

void Navigator::requestUltraSonic(uint8_t devices)
{
    // Check, if an ultra sonic measurement is pending already
    if (this->us_pending) printf("Navigator::requestUltraSonic() -> "
                                 "An ultra sonic measurement is pending already...dropping new request!\n");
    else
    {
        // Static variable which keeps track of the identifier number
        static uint32_t current_us_id = 0;
        // Initialize an ultra sonic request struct with values
        struct UltraSonic::Request request = {++current_us_id, 0, 0, 0, devices};

        // Write the ultra sonic request to the base process. True, if an error occurs
        if ((write(this->base_fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestUltraSonic", "ultra sonic request");
        // Set ultra sonic measurement pending to true
        else this->us_pending = true;
    }
}

void Navigator::requestOrientation()
{
    // Check, if an orientation measurement is pending already
    if (this->orientation_pending)
        printf("Navigator::requestOrientation() -> "
               "An orientation request is pending already...dropping new request!\n");
    else
    {
        // Static variable which keeps track of the identifier number
        static uint32_t current_orientation_id = 0;
        // Initialize an orientation request struct with values
        struct Orientation::Request request = {++current_orientation_id};

        // Write the orientation request to the base process. True, if an error occurs
        if ((write(this->base_fd, &Action::ORIENTATION, sizeof(Action::ORIENTATION)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestOrientation", "orientation");
        // Set orientation measurement pending to true
        else this->orientation_pending = true;
    }
}

void Navigator::requestSpeed()
{
    // Check, if a speed measurement is pending already
    if (this->speed_pending)
        printf("Navigator::requestSpeed() -> "
               "A speed request is pending already...dropping new request!\n");
    else
    {
        // Static variable which keeps track of the identifier number
        static uint32_t current_speed_id = 0;
        // Initialize an orientation request struct with values
        struct Speed::Request request = {++current_speed_id};

        // Write the speed request to the base process. True, if an error occurs
        if ((write(this->base_fd, &Action::SPEED, sizeof(Action::SPEED)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestSpeed", "speed request");
        // Set speed measurement pending to true
        else this->speed_pending = true;
    }
}

void Navigator::processIncomingMessage(uint8_t action)
{
    // Static variables which will be needed very often
    static ssize_t n;
    static struct UltraSonic::Response us_response;
    static struct Speed::Response speed_response;
    static struct Orientation::Response orientation_response;

    switch (action)
    {
        // The message is an ultra sonic response
        case Action::ULTRA_SONIC:
            // Get the actual response
            n = read(this->base_fd, &us_response, sizeof(us_response));
            // Check for errors
            if (n == -1) Navigator::printReadError("ultra sonic");
            // this is commented out to prevent the robot from overreacting to a bug (not pretty but it works)
//            else if (n != sizeof(us_response)) Navigator::printIncompleteError("ultra sonic");
            else if (n == sizeof(us_response))
            {
                // Process teh ultra sonic response. True, if the initial ultra sonic request failed. Make a new one
                if (!this->processUltraSonic(&us_response) && this->state == States::STARTING)
                {
                    printf("Navigator::processIncomingMessage() -> Making new ultra sonic request\n");
                    this->us_pending = false;
                    // Request an ultra sonic measurement on all devices
                    this->requestUltraSonic(0x0F);
                }
                else this->us_pending = false;
            }
            break;

        // The message is a speed response
        case Action::SPEED:
            // Get the actual response
            n = read(this->base_fd, &speed_response, sizeof(speed_response));
            // Check for errors
            if (n == -1) Navigator::printReadError("speed");
            else if (n != sizeof(speed_response)) Navigator::printIncompleteError("speed");
            // Process the speed response
            else this->processSpeed(&speed_response);
            this->speed_pending = false;
            break;

        // The message is an orientation response
        case Action::ORIENTATION:
            // Get the actual response
            n = read(this->base_fd, &orientation_response, sizeof(orientation_response));
            // Check for errors
            if (n == -1) Navigator::printReadError("orientation");
            else if (n != sizeof(orientation_response)) Navigator::printIncompleteError("orientation");
            // Process teh orientation response
            else this->processOrientation(&orientation_response);
            this->orientation_pending = false;
            break;

        // The message is a rotation complete notification
        case Action::ROTATION_COMPLETE:
            // Process the rotation response
            this->processRotationComplete();
            break;

        // The message is a start action
        case Action::START:
            // Request an ultra sonic measurement on all devices
            this->requestUltraSonic(0x0F);
            printf("Navigator::processIncomingMessage() -> Start signal received\n");
            break;

        // The action is a terminate action
        case Action::TERMINATE:
        {
            // Check, if a mapping process is connected. True, if a mapping process is connected
            if (this->mapping_fd != -1)
            {
                // Write the termination signal to the mapping process
                if (write(this->mapping_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
                    fprintf(stderr, "Navigator::processIncomingMessage() -> "
                                    "Error while write() to navigation process: %s\n", strerror(errno));
            }
        }
            break;

        // Unknown action identifier received
        default:
            printf("Navigator::processIncomingMessage() -> "
                   "Unexpected action from base process received: %d...dropping message!\n", action);
            // Is only here for bug prevention...
            this->us_pending = false;
            this->orientation_pending = false;
    }
}

void Navigator::processIncomingMessage(int fd, uint8_t action)
{
    switch (action)
    {
        // The action is an object detected action from the camera process
        case Action::OBJECT_DETECTED:
            // Stop all motors
            this->moveStraight(Motor::Direction::FORWARD, 0);
            // Acknowledge the object detected action
            if (write(fd, &Action::OBJECT_DETECTED_ACK, sizeof(Action::OBJECT_DETECTED_ACK)) == -1)
                Navigator::printWriteError("acknowledgeObjectDetection", "object detected acknowledge");
            // Check, if a mapping process is connected. True, if a mapping process is connected
            if (this->mapping_fd != -1)
            {
                // Write the termination signal to the mapping process
                if (write(this->mapping_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
                    Navigator::printWriteError("acknowledgeObjectDetection", "mapping terminate");
            }
            this->hunt_over = true;
            printf("Navigator::processIncomingMessage() -> Object detected!\n");
            // Call processRotationComplete() to rotate the whole time and notify the end of the hunt
            this->processRotationComplete();
            break;
        // The action is an identifier action from the mapping process
        case Action::IDENTIFIER_MAPPING:
            this->mapping_fd =  fd;
            printf("Navigator::processIncomingMessage() -> Mapping process identified\n");
            break;
        // Unknown action identifier received
        default:
            printf("Navigator::processIncomingMessage() -> "
                   "Unexpected action from arbitrary process received: %d...dropping message!\n", action);
    }
}

bool Navigator::processUltraSonic(struct UltraSonic::Response *response)
{
    // Check, if the hunt is over
    if (this->hunt_over) return false;
    // Check, if an ultra sonic measurement is pending
    if (!us_pending)
    {
        printf("Navigator::processUltraSonic -> "
               "Ultra sonic data received but no ultra sonic measurement is pending...discarding data\n");
        return false;
    }
    // Check, if the response contains invalid measurement values (because an error occurred)
    if ((response->distance_front < 0.) || (response->distance_rear < 0.) ||
             (response->distance_left < 0.) || (response->distance_right < 0.))
    {
        printf("Navigator::processUltraSonic -> Invalid ultra sonic data received...discarding data\n");
        printf("Front: %f, Back: %f, Left: %f, Right: %f\n", response->distance_front, response->distance_rear, response->distance_left, response->distance_right);
        return false;
    }
    // Check, if teh robot is moving backwards
    if (this->state == States::MOVING_BACKWARD)
    {
        // Check, if the distance to the rear is critical low.
        if (response->distance_rear < Navigation::Threshold::OVERALL_CM)
        {
            // Stop all motors
            this->moveStraight(Motor::Direction::FORWARD, 0);
            this->state = States::IDLING;
            // Make a new ultra sonic request
            this->requestUltraSonic(0x0F);
        }
        // Check, if the distance to the left is big enough to go there
        else if (response->distance_left > Navigation::Threshold::SIDE_CM)
            // Rotate to the left
            this->rotate(Rotation::Direction::LEFT, Navigation::Rotation::CHANGE_DIRECTION);
            // Check, if the distance to the right is big enough to go there
        else if (response->distance_right > Navigation::Threshold::SIDE_CM)
            // Rotate to the right
            this->rotate(Rotation::Direction::RIGHT, Navigation::Rotation::CHANGE_DIRECTION);
    }
    else
    {
        // Check, if the distance to the front is too low to move further to the front
        if (response->distance_front < Navigation::Threshold::STRAIGHT_CM)
        {
            // Check, if there is enough space to the right and there is more space, than to the left
            if ((response->distance_right > Navigation::Threshold::SIDE_CM) &&
                (response->distance_right > response->distance_left))
                // Rotate to the right
                this->rotate(Rotation::Direction::RIGHT, Navigation::Rotation::CHANGE_DIRECTION);
            // Check, if there is enough space to the left and there is more space, than to the right
            else if ((response->distance_left > Navigation::Threshold::SIDE_CM) &&
                     (response->distance_left > response->distance_right))
                // Rotate to the left
                this->rotate(Rotation::Direction::LEFT, Navigation::Rotation::CHANGE_DIRECTION);
            // Check, if the robot is moving backwards already (no other direction to go)
            else if (!(this->state == States::MOVING_BACKWARD))
            {
                // Start moving backward
                this->moveStraight(Motor::Direction::BACKWARD, Navigation::Speed::STRAIGHT);
                this->state = States::MOVING_BACKWARD;
            }
        }
        // Check, if the robot is moving forward
        else if (this->state == States::MOVING_FORWARD)
        {
            static float orientation_distance;
            static Side orientation;
            static uint8_t lower_speed, higher_speed;
            // Check, if the distance to teh right is smaller than the distance tot he left
            if (response->distance_right < response->distance_left)
            {
                orientation_distance = response->distance_right;
                orientation = Side::Right;
                lower_speed = Navigation::Speed::STRAIGHT - Navigation::Speed::SKEW_DIFFERENCE;
                higher_speed = Navigation::Speed::STRAIGHT + Navigation::Speed::SKEW_DIFFERENCE;
            }
            // The distance to the left is smaller than the distance to the right
            else
            {
                orientation_distance = response->distance_left;
                orientation = Side::Left;
                lower_speed = Navigation::Speed::STRAIGHT - Navigation::Speed::SKEW_DIFFERENCE;
                higher_speed = Navigation::Speed::STRAIGHT + Navigation::Speed::SKEW_DIFFERENCE;
            }
            // Check, if the distance to the closer side is very low
            if (orientation_distance < Navigation::Threshold::OVERALL_CM)
            {
                // Rotate a little to avoid a possible collision with a wall
                this->rotate((orientation == Side::Right) ?
                             Rotation::Direction::LEFT : Rotation::Direction::RIGHT, Navigation::Rotation::AVOID_COLLISION);
            }
            // Move a little skew away from a relative close object on one side
            else if (orientation_distance < Navigation::Threshold::MOVING_SIDE_MIN_CM)
                this->moveSkew(Motor::Direction::FORWARD, lower_speed, higher_speed);
        }
        // Check, if the robot is not moving forward already
        else if (!(this->state == States::MOVING_FORWARD))
        {
            // Start moving forward
            this->moveStraight(Motor::Direction::FORWARD, Navigation::Speed::STRAIGHT);
            this->state = States::MOVING_FORWARD;
        }
    }
    // Check, if the mapping process is connected. If it is connected, the ultra sonic data will be forwarded
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
    return true;
}

void Navigator::processOrientation(struct Orientation::Response *response)
{
    // Check, if the mapping process is connected. If it is connected, the ultra sonic data will be forwarded
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
}

void Navigator::processSpeed(struct Speed::Response *response)
{
    // Check, if the mapping process is connected. If it is connected, the ultra sonic data will be forwarded
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
}

void Navigator::processRotationComplete()
{
    // Rotate without an end, if the hunt is over
    if (this->hunt_over)
    {
        static bool rotate_left = true;

        if (rotate_left)
        {
            this->rotate(Rotation::Direction::LEFT, 180.);
            rotate_left = false;
        }
        else
        {
            this->rotate(Rotation::Direction::RIGHT, 180.);
            rotate_left = true;
        }
    }
    // The hunt is not over
    else
    {
        // Request an ultra sonic measurement
        this->requestUltraSonic(0x0F);
        // Check, if an ultra sonic measurement is connected
        if (this->mapping_fd != -1)
        {
            // Forward the rotation complete action to the mapping process
            if (write(this->mapping_fd, &Action::ROTATION_COMPLETE, sizeof(Action::ROTATION_COMPLETE)) == -1)
                Navigator::printWriteError("processRotationComplete", "rotation complete action");
        }
        // Actualize the current state
        this->state = Navigation::StateMachine::IDLING;
    }
}

void Navigator::moveStraight(uint8_t direction, uint8_t speed)
{
    // Initialize a motor command with the passed values
    struct Motor::Command command = {0x0F, direction, speed};

    // Send the motor command to the base process
    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
            Navigator::printWriteError("moveStraight", "motor command");
}

void Navigator::moveSkew(uint8_t direction, uint8_t speed_left, uint8_t speed_right)
{
    // Initialize a motor command for the left motors with the passed values
    struct Motor::Command command = {Motor::Device::FRONT_LEFT | Motor::Device::BACK_LEFT,
            direction, speed_left};

    // Send the motor command to the base process
    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
        Navigator::printWriteError("moveSkew", "motor command");
    // Alter the motor command for the right side
    command.devices = Motor::Device::FRONT_RIGHT | Motor::Device::BACK_RIGHT;
    command.duty_cycle = speed_right;
    // Send the motor command to the base process
    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
        Navigator::printWriteError("moveSkew", "motor command");
}

void Navigator::rotate(uint8_t direction, float degree)
{
    // Initialize a rotation request with the passed values
    struct Rotation::Command request = {direction, 0, 0, 0, degree};

    // Send the rotation request to the base process
    if ((write(this->base_fd, &Action::ROTATION, sizeof(Action::ROTATION)) == -1) ||
        (write(this->base_fd, &request, sizeof(request)) == -1))
        Navigator::printWriteError("rotate", "rotation request");
    else this->state = States::ROTATING;
}

int Navigator::getControlPipe()
{
    // Return the writing end of the internal control pipe
    return this->control_pipe[1];
}

void Navigator::printReadError(const std::string &part)
{
    fprintf(stderr, "Navigator::processIncomingMessage() -> "
                    "Error while reading %s response: %s...returning to work.\n",
            part.c_str(), strerror(errno));
}

void Navigator::printIncompleteError(const std::string &part)
{
    printf("Navigator::processIncomingMessage() -> "
           "Received incomplete %s response...returning to work.\n", part.c_str());
}

void Navigator::printWriteError(const std::string& func, const std::string &part)
{
    fprintf(stderr, "Navigator::%s() -> "
                    "Error while writing %s: %s\n", func.c_str(), part.c_str(), strerror(errno));
}

void Navigator::shareData(int fd, struct UltraSonic::Response *response)
{
    // Write the passed ultra sonic data to the passed file descriptor
    if ((write(fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
        (write(fd, response, sizeof(UltraSonic::Response)) == -1))
        Navigator::printWriteError("shareData", "ultra sonic response");
}

void Navigator::shareData(int fd, struct Orientation::Response *response)
{
    // Static variable to store a reference time
    static std::chrono::steady_clock::time_point reference_time;
    // Initialize an orientation response for the mapping process with the passed values and the time difference
    // to the last shared message
    struct Orientation::ResponseMapping response_map = {response->id, response->xyz[0], response->xyz[1],
            response->xyz[2], float(std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::steady_clock::now() - reference_time).count())};

    // Write the orientation response to the passed file descriptor
    if ((write(fd, &Action::ORIENTATION, sizeof(Action::ORIENTATION)) == -1) ||
        (write(fd, &response_map, sizeof(response_map)) == -1))
        Navigator::printWriteError("shareData", "orientation response");
    else reference_time = std::chrono::steady_clock::now();
}

void Navigator::shareData(int fd, struct Speed::Response *response)
{
    // Static variable to store a reference time
    static std::chrono::steady_clock::time_point reference_time;
    // Initialize a speed response for the mapping process with the passed values and the time difference
    // to the last shared message
    struct Speed::ResponseMapping response_map = {response->id, response->value, float(std::chrono::duration_cast<
            std::chrono::milliseconds>(std::chrono::steady_clock::now() - reference_time).count())};

    // Write the speed response to the passed file descriptor
    if ((write(fd, &Action::SPEED, sizeof(Action::SPEED)) == -1) ||
        (write(fd, &response_map, sizeof(response_map)) == -1))
        Navigator::printWriteError("shareData", "speed response");
    else reference_time = std::chrono::steady_clock::now();
}

