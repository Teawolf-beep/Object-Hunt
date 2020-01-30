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
    if (base_fd < 0) throw NavigatorException("ConnectionManagerException() -> "
                                                      "Error while connecting to base socket!");
    if (pipe(this->control_pipe))
        throw NavigatorException("Navigator() -> Error while creating the main pipe: " +
                                 std::string(std::strerror(errno)));
    this->connection_manager->setNavigationControlPipe(this->control_pipe[1]);
}

Navigator::~Navigator()
{
    printf("~Navigator() -> Closing base socket...\n");
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

    pfd[0].fd = this->control_pipe[0];
    pfd[0].events = POLLIN;
    pfd[1].fd = this->base_fd;
    pfd[1].events = POLLIN;

    while (!exit_condition)
    {
        if (refresh_connections)
        {
            const std::vector<int> connections = this->connection_manager->getConnections();

            for (connection_count = 2; connection_count < (connections.size()+2); ++connection_count)
            {
                if (connection_count >= Socket::Settings::MAX_CONNECTIONS)
                {
                    printf("Navigator::run() -> The number of established connections exhibits the maximum "
                           "number of connections. Cannot monitor all file descriptor.\nPlease remove some connections.\n");
                    break;
                }
                pfd[connection_count].fd = connections[connection_count-2];
                pfd[connection_count].events = POLLIN;
            }
            refresh_connections = false;
            printf("Navigator::run() -> Listening on %d file descriptors.\n", connection_count+1);
        }
        processing_time = int(Navigation::Timeout::GENERAL_MS - std::chrono::duration_cast<std::chrono::milliseconds>
                (std::chrono::steady_clock::now() - reference_time).count());

        timeout = (processing_time > 0) ? processing_time : 0;
        ret_poll = poll(pfd, connection_count, timeout);
        if (ret_poll == -1)
        {
            fprintf(stderr, "Navigator::run() -> "
                            "Error while poll(): %s\n", strerror(errno));
        }
        else if (ret_poll == 0)
        {
            reference_time = std::chrono::steady_clock::now();
            if ((this->state == States::MOVING_FORWARD) ||
                (this->state == States::MOVING_BACKWARD))
            {
                this->requestUltraSonic(0x0F);
                this->requestSpeed();
                this->requestOrientation();
            }
        }
        else if (pfd[0].revents &  POLLIN)
        {
            n = read(this->control_pipe[0], &exit_condition, sizeof(exit_condition));
            if (n == -1)
            {
                fprintf(stderr, "Navigator::run() -> "
                                "Error while read() on control pipe: %s\n", strerror(errno));
                return -1;
            }
            if (exit_condition) printf("Navigator::run() -> Termination signal received\n");
            else refresh_connections = true;
            continue;
        }
        else if (pfd[1].revents & POLLIN)
        {
            n = read(pfd[1].fd, &action, sizeof(action));
            if (n == -1)
            {
                fprintf(stderr, "Navigator::run() -> "
                                "Error while read() on base pipe: %s...returning to work.\n", strerror(errno));
                continue;
            }
            else if (n == 0)
            {
                printf("Navigator::run() -> Fatal error! Lost connection to the base process...Aborting!\n");
                return -2;
            }
            else this->processIncomingMessage(action);
        }
        else
        {
            for (i = 1; i < connection_count; ++i)
            {
                if (pfd[i].revents & POLLIN)
                {
                    n = read(pfd[i].fd, &action, sizeof(action));
                    if (n == -1)
                    {
                        fprintf(stderr, "Navigator::run() -> "
                                        "Error while read(): %s...returning to work.\n", strerror(errno));
                        continue;
                    }
                    else if (n == 0)
                    {
                        connection_manager->removeConnection(pfd[i].fd);
                        if (pfd[i].fd == this->mapping_fd)
                        {
                            this->mapping_fd = -1;
                            printf("Navigator::run() -> Connection to mapping process lost\n");
                        }
                        refresh_connections = true;
                        write(this->control_pipe[1], &exit_condition, sizeof(exit_condition));
                    }
                    else this->processIncomingMessage(pfd[i].fd, action);
                }
            }
        }
    }
    return 0;
}

void Navigator::requestUltraSonic(uint8_t devices)
{
    if (this->us_pending) printf("Navigator::requestUltraSonic() -> "
                                 "An ultra sonic measurement is pending already...dropping new request!\n");
    else
    {
        static uint32_t current_us_id = 0;
        struct UltraSonic::Request request = {++current_us_id, 0, 0, 0, devices};

        if ((write(this->base_fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestUltraSonic", "ultra sonic request");
        else this->us_pending = true;
    }
}

void Navigator::requestOrientation()
{
    if (this->orientation_pending)
        printf("Navigator::requestOrientation() -> "
               "An orientation request is pending already...dropping new request!\n");
    else
    {
        static uint32_t current_orientation_id = 0;
        struct Orientation::Request request = {++current_orientation_id};

        if ((write(this->base_fd, &Action::ORIENTATION, sizeof(Action::ORIENTATION)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestOrientation", "orientation");
        else this->orientation_pending = true;
    }
}

void Navigator::requestSpeed()
{
    if (this->speed_pending)
        printf("Navigator::requestSpeed() -> "
               "A speed request is pending already...dropping new request!\n");
    else
    {
        static uint32_t current_speed_id = 0;
        struct Speed::Request request = {++current_speed_id};

        if ((write(this->base_fd, &Action::SPEED, sizeof(Action::SPEED)) == -1) ||
            (write(this->base_fd, &request, sizeof(request)) == -1))
            Navigator::printWriteError("requestSpeed", "speed request");
        else this->speed_pending = true;
    }
}

void Navigator::processIncomingMessage(uint8_t action)
{
    static ssize_t n;
    static struct UltraSonic::Response us_response;
    static struct Speed::Response speed_response;
    static struct Orientation::Response orientation_response;

    switch (action)
    {
        case Action::ULTRA_SONIC:
            n = read(this->base_fd, &us_response, sizeof(us_response));

            if (n == -1) Navigator::printReadError("ultra sonic");
//            else if (n != sizeof(us_response)) Navigator::printIncompleteError("ultra sonic");
            else if (n == sizeof(us_response))
            {
                if (!this->processUltraSonic(&us_response) && this->state == States::STARTING)
                {
                    printf("Navigator::processIncomingMessage() -> Making new ultra sonic request\n");
                    this->us_pending = false;
                    this->requestUltraSonic(0x0F);
                }
                else this->us_pending = false;
            }
            break;

        case Action::SPEED:
            n = read(this->base_fd, &speed_response, sizeof(speed_response));

            if (n == -1) Navigator::printReadError("speed");
            else if (n != sizeof(speed_response)) Navigator::printIncompleteError("speed");
            else this->processSpeed(&speed_response);
            this->speed_pending = false;
            break;

        case Action::ORIENTATION:
            n = read(this->base_fd, &orientation_response, sizeof(orientation_response));

            if (n == -1) Navigator::printReadError("orientation");
            else if (n != sizeof(orientation_response)) Navigator::printIncompleteError("orientation");
            else this->processOrientation(&orientation_response);
            this->orientation_pending = false;
            break;

        case Action::ROTATION_COMPLETE:
            this->processRotationComplete();
            break;

        case Action::START:
            this->requestUltraSonic(0x0F);
            printf("Navigator::processIncomingMessage() -> Start signal received\n");
            break;

        case Action::TERMINATE:
        {
            if (this->mapping_fd != -1)
            {
                if (write(this->mapping_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
                    fprintf(stderr, "Navigator::processIncomingMessage() -> "
                                    "Error while write() to navigation process: %s\n", strerror(errno));
            }
        }
            break;

        default:
            printf("Navigator::processIncomingMessage() -> "
                   "Unexpected action from base process received: %d...dropping message!\n", action);
            this->us_pending = false;
            this->orientation_pending = false;
    }
}

void Navigator::processIncomingMessage(int fd, uint8_t action)
{
    switch (action)
    {
        case Action::OBJECT_DETECTED:
            this->moveStraight(Motor::Direction::FORWARD, 0);
            if (write(fd, &Action::OBJECT_DETECTED_ACK, sizeof(Action::OBJECT_DETECTED_ACK)) == -1)
                Navigator::printWriteError("acknowledgeObjectDetection", "object detected acknowledge");
            if (this->mapping_fd != -1)
            {
                if (write(this->mapping_fd, &Action::TERMINATE, sizeof(Action::TERMINATE)) == -1)
                    Navigator::printWriteError("acknowledgeObjectDetection", "mapping terminate");
            }
            this->hunt_over = true;
            printf("Navigator::processIncomingMessage() -> Object detected!\n");
            this->processRotationComplete();
            break;

        case Action::IDENTIFIER_MAPPING:
            this->mapping_fd =  fd;
            printf("Navigator::processIncomingMessage() -> Mapping process identified\n");
            break;

        default:
            printf("Navigator::processIncomingMessage() -> "
                   "Unexpected action from arbitrary process received: %d...dropping message!\n", action);
    }
}

bool Navigator::processUltraSonic(struct UltraSonic::Response *response)
{
    if (this->hunt_over) return false;
    if (!us_pending)
    {
        printf("Navigator::processUltraSonic -> "
               "Ultra sonic data received but no ultra sonic measurement is pending...discarding data\n");
        return false;
    }
    if ((response->distance_front < 0.) || (response->distance_rear < 0.) ||
             (response->distance_left < 0.) || (response->distance_right < 0.))
    {
        printf("Navigator::processUltraSonic -> Invalid ultra sonic data received...discarding data\n");
        printf("Front: %f, Back: %f, Left: %f, Right: %f\n", response->distance_front, response->distance_rear, response->distance_left, response->distance_right);
        return false;
    }

//    printf("Front: %f, Back: %f, Left: %f, Right: %f\n", response->distance_front, response->distance_rear, response->distance_left, response->distance_right);

    if (this->state == States::MOVING_BACKWARD)
    {
        if (response->distance_rear < Navigation::Threshold::OVERALL_CM)
        {
            this->moveStraight(Motor::Direction::FORWARD, 0);
            this->state = States::IDLING;
            this->requestUltraSonic(0x0F);
        }
        else if (response->distance_left > Navigation::Threshold::SIDE_CM)
            this->rotate(Rotation::Direction::LEFT, Navigation::Rotation::CHANGE_DIRECTION);
        else if (response->distance_right > Navigation::Threshold::SIDE_CM)
            this->rotate(Rotation::Direction::RIGHT, Navigation::Rotation::CHANGE_DIRECTION);
    }
    else
    {
        if (response->distance_front < Navigation::Threshold::STRAIGHT_CM)
        {
            if ((response->distance_right > Navigation::Threshold::SIDE_CM) &&
                (response->distance_right > response->distance_left))
                this->rotate(Rotation::Direction::RIGHT, Navigation::Rotation::CHANGE_DIRECTION);

            else if ((response->distance_left > Navigation::Threshold::SIDE_CM) &&
                     (response->distance_left > response->distance_right))
                this->rotate(Rotation::Direction::LEFT, Navigation::Rotation::CHANGE_DIRECTION);

            else if (!(this->state == States::MOVING_BACKWARD))
            {
                this->moveStraight(Motor::Direction::BACKWARD, Navigation::Speed::STRAIGHT);
                this->state = States::MOVING_BACKWARD;
            }
        }
        else if (this->state == States::MOVING_FORWARD)
        {
            static float orientation_distance;
            static Side orientation;
            static uint8_t lower_speed, higher_speed;

            if (response->distance_right < response->distance_left)
            {
                orientation_distance = response->distance_right;
                orientation = Side::Right;
                lower_speed = Navigation::Speed::STRAIGHT - Navigation::Speed::SKEW_DIFFERENCE;
                higher_speed = Navigation::Speed::STRAIGHT + Navigation::Speed::SKEW_DIFFERENCE;
            }
            else
            {
                orientation_distance = response->distance_left;
                orientation = Side::Left;
                lower_speed = Navigation::Speed::STRAIGHT - Navigation::Speed::SKEW_DIFFERENCE;
                higher_speed = Navigation::Speed::STRAIGHT + Navigation::Speed::SKEW_DIFFERENCE;
            }

            if (orientation_distance < Navigation::Threshold::OVERALL_CM)
            {
                this->rotate((orientation == Side::Right) ?
                             Rotation::Direction::LEFT : Rotation::Direction::RIGHT, Navigation::Rotation::AVOID_COLLISION);
            }
            else if (orientation_distance < Navigation::Threshold::MOVING_SIDE_MIN_CM)
                this->moveSkew(Motor::Direction::FORWARD, lower_speed, higher_speed);
        }
        else if (!(this->state == States::MOVING_FORWARD))
        {
            this->moveStraight(Motor::Direction::FORWARD, Navigation::Speed::STRAIGHT);
            this->state = States::MOVING_FORWARD;
        }
    }
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
    return true;
}

void Navigator::processOrientation(struct Orientation::Response *response)
{
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
}

void Navigator::processSpeed(struct Speed::Response *response)
{
    if (this->mapping_fd != -1) Navigator::shareData(this->mapping_fd, response);
}

void Navigator::processRotationComplete()
{
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
    else
    {
        this->requestUltraSonic(0x0F);
        if (this->mapping_fd != -1)
        {
            if (write(this->mapping_fd, &Action::ROTATION_COMPLETE, sizeof(Action::ROTATION_COMPLETE)) == -1)
                Navigator::printWriteError("processRotationComplete", "rotation complete action");
        }
        this->state = Navigation::StateMachine::IDLING;
    }
}

void Navigator::moveStraight(uint8_t direction, uint8_t speed)
{
    struct Motor::Command command = {0x0F, direction, speed};

    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
            Navigator::printWriteError("moveStraight", "motor command");
}

void Navigator::moveSkew(uint8_t direction, uint8_t speed_left, uint8_t speed_right)
{
    struct Motor::Command command = {Motor::Device::FRONT_LEFT | Motor::Device::BACK_LEFT,
            direction, speed_left};

    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
        Navigator::printWriteError("moveSkew", "motor command");

    command.devices = Motor::Device::FRONT_RIGHT | Motor::Device::BACK_RIGHT;
    command.duty_cycle = speed_right;

    if ((write(this->base_fd, &Action::MOTOR, sizeof(Action::MOTOR)) == -1) ||
        (write(this->base_fd, &command, sizeof(command)) == -1))
        Navigator::printWriteError("moveSkew", "motor command");
}

void Navigator::rotate(uint8_t direction, float degree)
{
    struct Rotation::Command request = {direction, 0, 0, 0, degree};

    if ((write(this->base_fd, &Action::ROTATION, sizeof(Action::ROTATION)) == -1) ||
        (write(this->base_fd, &request, sizeof(request)) == -1))
        Navigator::printWriteError("rotate", "rotation request");
    else this->state = States::ROTATING;
}

int Navigator::getControlPipe()
{
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
//    printf("Sending ultra sonic data: Front: %f, Rear: %f, Left %f, Right %f\n", response->distance_front, response->distance_back, response->distance_left, response->distance_right);
    if ((write(fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
        (write(fd, response, sizeof(UltraSonic::Response)) == -1))
        Navigator::printWriteError("shareData", "ultra sonic response");
}

void Navigator::shareData(int fd, struct Orientation::Response *response)
{
    static std::chrono::steady_clock::time_point reference_time;
    struct Orientation::ResponseMapping response_map = {response->id, response->xyz[0], response->xyz[1],
            response->xyz[2], float(std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::steady_clock::now() - reference_time).count())};

//    printf("Sending orientation data: X: %f, Y: %f, Z: %f\n", response->xyz[0], response->xyz[1], response->xyz[2]);
    if ((write(fd, &Action::ORIENTATION, sizeof(Action::ORIENTATION)) == -1) ||
        (write(fd, &response_map, sizeof(response_map)) == -1))
        Navigator::printWriteError("shareData", "orientation response");
    else reference_time = std::chrono::steady_clock::now();
}

void Navigator::shareData(int fd, struct Speed::Response *response)
{
    static std::chrono::steady_clock::time_point reference_time;
    struct Speed::ResponseMapping response_map = {response->id, response->value, float(std::chrono::duration_cast<
            std::chrono::milliseconds>(std::chrono::steady_clock::now() - reference_time).count())};

//    printf("Sending speed data: %f\n", response->speed);
    if ((write(fd, &Action::SPEED, sizeof(Action::SPEED)) == -1) ||
        (write(fd, &response_map, sizeof(response_map)) == -1))
        Navigator::printWriteError("shareData", "speed response");
    else reference_time = std::chrono::steady_clock::now();
}

