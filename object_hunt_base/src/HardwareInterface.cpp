//
// Created by robert on 04.12.19.
//

// System includes
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <vector>
#include <cerrno>
#include <chrono>

// Project includes
#include "HardwareInterface.h"
#include "ObjectHuntDefinitions.h"
#include "GPIO.h"
#include "HC_SR04.h"
#include "TCST_1103.h"
#include "GearMotor.h"
#include "VMA_401.h"
#include "BNO_055.h"

// Constructor. Initializes members via member initializer list.
HardwareInterface::HardwareInterface(std::shared_ptr<GPIO> gpio)
        : gpio(gpio),
          motor_fr(std::make_unique<GearMotor>(gpio, Motor::Pin::FR_FORWARD, Motor::Pin::FR_BACKWARD, Motor::PWM_FREQUENCY)),
          motor_fl(std::make_unique<GearMotor>(gpio, Motor::Pin::FL_FORWARD, Motor::Pin::FL_BACKWARD, Motor::PWM_FREQUENCY)),
          motor_rr(std::make_unique<GearMotor>(gpio, Motor::Pin::RR_FORWARD, Motor::Pin::RR_BACKWARD, Motor::PWM_FREQUENCY)),
          motor_rl(std::make_unique<GearMotor>(gpio, Motor::Pin::RL_FORWARD, Motor::Pin::RL_BACKWARD, Motor::PWM_FREQUENCY)),
          bno(std::make_unique<BNO_055>(BNO055::ADDRESS_A)),
          stepper(std::make_unique<VMA_401>(gpio, Stepper::Pin::ONE, Stepper::Pin::TWO, Stepper::Pin::THREE, Stepper::Pin::FOUR)),
          speed(0),
          us_running(false)
{
    // Create internal pipes
    if (pipe(this->ultra_sonic_pipe))
    {
        throw HardwareInterfaceException("HardwareInterface() -> Error while creating the ultra sonic pipe: " +
                              std::string(std::strerror(errno)));
    }
    if (pipe(this->speed_pipe))
    {
        throw HardwareInterfaceException("HardwareInterface() -> Error while creating the revolution pipe: " +
                              std::string(std::strerror(errno)));
    }
    if (pipe(this->rotation_pipe))
    {
        throw HardwareInterfaceException("HardwareInterface() -> Error while creating the rotation pipe: " +
                                       std::string(std::strerror(errno)));
    }
    // Set the desired interrupt options
    std::vector<InterruptDescriptor> interrupts;

    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::FRONT_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::REAR_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::LEFT_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::RIGHT_ECHO, Edge::BOTH});

    interrupts.push_back(InterruptDescriptor {Revolution::Pin::FRONT_RIGHT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::FRONT_LEFT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::REAR_LEFT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::REAR_RIGHT, Edge::FALLING});

    // Set all interrupt kernel driver at once
    if (!this->gpio->setMultipleKernelDriver(interrupts, PullState::OFF))
        throw HardwareInterfaceException("HardwareInterface() -> Error while setting GPIO kernel driver!");
    // Initialize the ultra sonic sensors
    this->us_front = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::FRONT, UltraSonic::Pin::FRONT_TRIGGER,
                                               UltraSonic::Pin::FRONT_ECHO,pollfd {interrupts[0].fd, POLLPRI});
    this->us_rear = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::REAR, UltraSonic::Pin::REAR_TRIGGER,
                                              UltraSonic::Pin::REAR_ECHO, pollfd {interrupts[1].fd, POLLPRI});
    this->us_left = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::LEFT, UltraSonic::Pin::LEFT_TRIGGER,
                                              UltraSonic::Pin::LEFT_ECHO,pollfd {interrupts[2].fd, POLLPRI});
    this->us_right = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::RIGHT, UltraSonic::Pin::RIGHT_TRIGGER,
                                               UltraSonic::Pin::RIGHT_ECHO, pollfd {interrupts[3].fd, POLLPRI});
    // Initialize the revolution sensors
    this->rev_fr = std::make_unique<TCST_1103>(gpio, Revolution::Pin::FRONT_RIGHT, interrupts[4].fd);
    this->rev_fl = std::make_unique<TCST_1103>(gpio, Revolution::Pin::FRONT_LEFT, interrupts[5].fd);
    this->rev_rr = std::make_unique<TCST_1103>(gpio, Revolution::Pin::REAR_LEFT, interrupts[6].fd);
    this->rev_rl = std::make_unique<TCST_1103>(gpio, Revolution::Pin::REAR_RIGHT, interrupts[7].fd);
    // Make the enable motor pin to an output
    this->gpio->makeOutput(Motor::ENABLE_ALL);
    // Set the operation mode of the smart movement sensor
    bno->setOperationMode(BNO055::OperationMode::IMUPLUS);
    // Initialize the speed measure agent thread (will run until destruction of this object)
    this->speed_agent = std::thread(&HardwareInterface::updateSpeed, this);
    this->enableMotors();
}

HardwareInterface::~HardwareInterface()
{
    // Stop all motors
    this->stopMotors();
    // Disable all motors
    this->disableMotors();
    // Terminate the speed measure agent
    this->terminateSpeedAgent();
    // Joint he speed measure agent, if possible
    if (this->us_agent.joinable()) this->us_agent.join();
}

void HardwareInterface::enableMotors()
{
    // Enable all motors
    this->gpio->setPin(Motor::ENABLE_ALL);
}

void HardwareInterface::disableMotors()
{
    // Disable all motors
    this->gpio->clearPin(Motor::ENABLE_ALL);
}

bool HardwareInterface::areMotorsEnabled()
{
    // Return whether the motors are enabled or not
    return this->gpio->readPin(Motor::ENABLE_ALL);
}

bool HardwareInterface::moveMotor(struct Motor::Command *command)
{
    // Check the desired direction
    if ((command->direction != Motor::Direction::FORWARD) && (command->direction != Motor::Direction::BACKWARD))
    {
        printf("HardwareInterface::moveMotor() -> "
               "Unknown direction command (%d) received"
               "...ignoring command.\n", command->direction);
        // Return false to indicate an invalid command
        return false;
    }
    // True, if the motor in front right shall move
    if (command->devices & Motor::Device::FRONT_RIGHT)
    {
        // Stop the motor, if the duty cycle equals 0
        if (command->duty_cycle == 0) this->motor_fr->stop();
        // Start the motor moving thread
        else this->motor_fr->startMoving(command->direction, command->duty_cycle);
    }
    // True, if the motor in front left shall move
    if (command->devices & Motor::Device::FRONT_LEFT)
    {
        // Stop the motor, if the duty cycle equals 0
        if (command->duty_cycle == 0) this->motor_fl->stop();
        // Start the motor moving thread
        else this->motor_fl->startMoving(command->direction, command->duty_cycle);
    }
    // True, if the motor in rear right shall move
    if (command->devices & Motor::Device::BACK_RIGHT)
    {
        // Stop the motor, if the duty cycle equals 0
        if (command->duty_cycle == 0) this->motor_rr->stop();
        // Start the motor moving thread
        else this->motor_rr->startMoving(command->direction, command->duty_cycle);
    }
    // True, if the motor in rear right shall move
    if (command->devices & Motor::Device::BACK_LEFT)
    {
        // Stop the motor, if the duty cycle equals 0
        if (command->duty_cycle == 0) this->motor_rl->stop();
        // Start the motor moving thread
        else this->motor_rl->startMoving(command->direction, command->duty_cycle);
    }
    // Return true, to indicate a valid command
    return true;
}

bool HardwareInterface::moveStepper(struct Stepper::Command *command)
{
    // Check the number of desired steps
    if (command->steps > Stepper::MAX_STEPS)
    {
        printf("HardwareInterface::moveStepper() -> "
               "Step number is higher than maximum permitted number"
               "...ignoring command.\n");
        // Return false to indicate an invalid command
        return false;
    }
    // Check the desired direction
    else if ((command->direction != Stepper::Direction::RIGHT) && (command->direction != Stepper::Direction::LEFT))
    {
        printf("HardwareInterface::moveStepper() -> "
               "Unknown direction command (%d)"
               "...ignoring command.\n", command->direction);
        // Return false to indicate an invalid command
        return false;
    }
    // Stop the stepper, if the desired steps equal zero
    else if (command->steps == 0) this->stepper->stop();
    // Trigger the stepper moving thread with the passed command options
    else this->stepper->startMoving(command->direction, command->steps, Stepper::IDLE_TIME);
    return true;
}

void HardwareInterface::startUltraSonic(int response_fd, struct UltraSonic::Request *request)
{
    // Check, if an ultra sonic measurement is running already
    if (this->us_running)
    {
        // Initialize the response (-1 means a problem occurred)
        struct UltraSonic::Response response = {request->id, -1.,
                                                -1., -1., -1.};
        struct pollfd pfd_response = {response_fd, POLLOUT};
        // Lock the write mutex with a scoped lock
        std::lock_guard<std::mutex> lock(this->write_mutex);
        // Check, if the file descriptor can be written
        if (poll(&pfd_response, 1, -1) == -1)
            fprintf(stderr, "HardwareInterface::startUltraSonic() -> "
                            "Error while poll() on navigation fd: %s\n", strerror(errno));
        // Write to the file descriptor
        else if ((write(response_fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
                 (write(response_fd, &response, sizeof(response)) == -1))
        {
            fprintf(stderr, "HardwareInterface::startUltraSonic() -> "
                            "Error while write() on navigation fd: %s\n", strerror(errno));
        }
        printf("HardwareInterface::startUltraSonic() -> Unable to start new ultra "
               "sonic measurement, running already\n");
    }
    else
    {
        // Join the ultra sonic agent, if possible
        if (this->us_agent.joinable()) this->us_agent.join();
        // trigger a new ultra sonic measurement
        this->us_agent = std::thread(&HardwareInterface::measureUltraSonic,
                                     this, response_fd, request);
    }
}

void HardwareInterface::measureUltraSonic(int response_fd, struct UltraSonic::Request* request)
{
    this->us_running = true;

    uint8_t measurements = 0;
    ssize_t n;
    // Initialize a pollfd with the response file descriptor and an output event
    struct pollfd pfd = {response_fd, POLLOUT};
    // Initial the response struct with values (-1 means a problem occurred). Will be overwritten later, if everything works
    struct UltraSonic::Response response = {request->id, -1., -1., -1., -1.};
    struct InternalUSResponse internal_response;

    // Check, if the sensor in the front is requested
    if (request->devices & UltraSonic::Device::FRONT)
    {
        // Trigger an ultra sonic measurement
        this->us_front->measure(this->ultra_sonic_pipe[1]);
        // Increment the measurement count
        ++measurements;
    }
    // Check, if the sensor in the rear is requested
    if (request->devices & UltraSonic::Device::REAR)
    {
        // Trigger an ultra sonic measurement
        this->us_rear->measure(this->ultra_sonic_pipe[1]);
        // Increment the measurement count
        ++measurements;
    }
    // Check, if the sensor in the left is requested
    if (request->devices & UltraSonic::Device::LEFT)
    {
        // Trigger an ultra sonic measurement
        this->us_left->measure(this->ultra_sonic_pipe[1]);
        // Increment the measurement count
        ++measurements;
    }
    // Check, if the sensor in the right is requested
    if (request->devices & UltraSonic::Device::RIGHT)
    {
        // Trigger an ultra sonic measurement
        this->us_right->measure(this->ultra_sonic_pipe[1]);
        // Increment the measurement count
        ++measurements;
    }
    // Check, if any measurement was requested at all
    if (measurements < 1) printf("HardwareInterface::measureUltraSonic() -> "
                                 "Invalid device sequence (%02X)...Ignoring request", request->devices);
    // Collect all measurement results in a loop
    for (uint8_t i = 0; i < measurements; ++i)
    {
        n = read(this->ultra_sonic_pipe[0], &internal_response, sizeof(internal_response));
        if (n == -1) fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                                     "Error while read(): %s\n", strerror(errno));
        else if (internal_response.device == UltraSonic::Device::FRONT)
            response.distance_front = internal_response.distance;
        else if (internal_response.device == UltraSonic::Device::REAR)
            response.distance_rear = internal_response.distance;
        else if (internal_response.device == UltraSonic::Device::LEFT)
            response.distance_left = internal_response.distance;
        else if (internal_response.device == UltraSonic::Device::RIGHT)
            response.distance_right = internal_response.distance;
    }
    // Lock the write mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->write_mutex);
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while poll(): %s\n", strerror(errno));
    // Write to the file descriptor
    else if ((write(response_fd, &Action::ULTRA_SONIC, sizeof(Action::ULTRA_SONIC)) == -1) ||
             (write(response_fd, &response, sizeof(response)) == -1))
    {
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while write(): %s\n", strerror(errno));
    }
    this->us_running = false;
}

void HardwareInterface::getRevolution(int response_fd, struct Revolution::Request *request)
{
    // Initialize a pollfd with the response file descriptor and an output event
    struct pollfd pfd = {response_fd, POLLOUT};
    // Initial the response struct with values (-1 means a problem occurred). Will be overwritten later, if everything works
    struct Revolution::Response response = {request->id, -1, -1,
            -1, -1};
    // Overwrite the fields of the response struct
    if (request->devices & Revolution::Device::FRONT_RIGHT) response.count_front_right = this->rev_fr->getCounter();
    if (request->devices & Revolution::Device::FRONT_LEFT) response.count_front_left = this->rev_fl->getCounter();
    if (request->devices & Revolution::Device::REAR_RIGHT) response.count_rear_right = this->rev_rr->getCounter();
    if (request->devices & Revolution::Device::REAR_LEFT) response.count_rear_left = this->rev_rl->getCounter();
    // Lock the write mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->write_mutex);
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while poll(): %s\n", strerror(errno));
    // Write to the file descriptor
    else if ((write(response_fd, &Action::REVOLUTION, sizeof(Action::REVOLUTION)) == -1) ||
             (write(response_fd, &response, sizeof(response)) == -1))
    {
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while write(): %s\n", strerror(errno));
    }
}

bool HardwareInterface::getSpeed(int response_fd, uint32_t id)
{
    // Initialize a pollfd with the response file descriptor and an output event
    struct pollfd pfd = {response_fd, POLLOUT};
    // Initial the response struct with the passed id and the current speed value
    struct Speed::Response response = {id, this->speed};

    // Lock the write mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->write_mutex);
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::getSpeed() -> "
                        "Error while poll(): %s\n", strerror(errno));
        return false;
    }
    // Write to the file descriptor
    if ((write(response_fd, &Action::SPEED, sizeof(Action::SPEED)) == -1) ||
             (write(response_fd, &response, sizeof(response)) == -1))
    {
        fprintf(stderr, "HardwareInterface::getSpeed() -> "
                        "Error while write(): %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool HardwareInterface::getOrientation(int response_fd, uint32_t id)
{
    // Initialize a pollfd with the response file descriptor and an output event
    struct pollfd pfd = {response_fd, POLLOUT};
    // Initial the response struct with values (1000 means a problem occurred). Will be overwritten later, if everything works
    struct Orientation::Response response = {id, 1000., 1000., 1000.};
    bool ret = false;

    // Get the current orientation from the smart movement sensor
    if (this->bno->getEuler(response.xyz)) ret = true;
    // Lock the write mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->write_mutex);
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::getOrientation() -> "
                        "Error while poll(): %s\n", strerror(errno));
        ret = false;
    }
    // Write to the file descriptor
    else if ((write(response_fd, &Action::ORIENTATION, sizeof(Action::ORIENTATION)) == -1) ||
             (write(response_fd, &response, sizeof(response)) == -1))
    {
        fprintf(stderr, "HardwareInterface::getOrientation() -> "
                        "Error while write(): %s\n", strerror(errno));
        ret = false;
    }
    return ret;
}

void HardwareInterface::updateSpeed()
{
    bool exit_condition = false;
    // Initialize a pollfd with the internal pipe and an input event
    struct pollfd pfd = {this->speed_pipe[0], POLLIN};
    // Initialize further variables
    std::chrono::steady_clock::time_point time_reference;
    float prev_rev_count, current_rev_count = 0, change_rev_count;
    uint8_t current_direction;

    // Will be left, when the exit condition is true (will happen at object destruction)
    while (!exit_condition)
    {
        // Store the current revolution count for the next time
        prev_rev_count = current_rev_count;
        // Get the current revolution mean value
        current_rev_count = this->getRevolutionMedian();
        // Get the direction of each motor
        current_direction = this->motor_fl->getDirection() & this->motor_fr->getDirection() &
                            this->motor_rl->getDirection() & this->motor_rr->getDirection();

        switch (current_direction)
        {
            // All motors are moving forwards
            case Motor::Direction::FORWARD:
                change_rev_count = current_rev_count - prev_rev_count;
                break;

            // All motors are moving backwards
            case Motor::Direction::BACKWARD:
                change_rev_count = prev_rev_count - current_rev_count;
                break;

            // No motor is moving or they are moving in different directions
            default:
                change_rev_count = 0;
        }
        // Calculate the current speed
        this->speed = change_rev_count * Speed::DISTANCE_PER_COUNT_CM /
                       std::chrono::duration<float>(std::chrono::steady_clock::now() - time_reference).count();
        // Get a time reference for the next time
        time_reference = std::chrono::steady_clock::now();
        // Wait on the internal pipe until the timer expires or a message comes in
        if (poll(&pfd, 1, Speed::MEASUREMENT_INTERVAL_MS) == -1)
        {
            fprintf(stderr, "HardwareInterface::updateSpeed() -> "
                            "Error while poll(): %s\n", strerror(errno));
            return;
        }
        // True, if a message came in
        else if (pfd.revents & POLLIN)
        {
            // Update the value of exit_condition
            ssize_t n = read(this->speed_pipe[0], &exit_condition, sizeof(exit_condition));
            if (n == -1)
            {
                fprintf(stderr, "HardwareInterface::updateSpeed() -> "
                                "Error while read() on value pipe: %s\n", strerror(errno));
                return;
            }
        }
    }
}

float HardwareInterface::getRevolutionMedian()
{
    // Calculate the mean value of all revolution sensors
    return float((this->rev_fr->getCounter() + this->rev_fl->getCounter() +
                  this->rev_rr->getCounter() + this->rev_rl->getCounter())/4.);
}

void HardwareInterface::terminateSpeedAgent()
{
    // Create an exit condition variable with value true
    bool exit_condition = true;
    // Write the exit condition variable to the internal speed pipe
    write(this->speed_pipe[1], &exit_condition, sizeof(bool));
    // Wait for the speed thread to terminate and join it
    this->speed_agent.join();
}

void HardwareInterface::startRotation(int response_fd, struct Rotation::Command *request)
{
    this->rotation_running = true;
    // Initialize a pollfd with the internal rotation pipe and an input event
    struct pollfd pfd = {this->rotation_pipe[0], POLLIN};
    // initialize further variables
    float target_rotation;
    float previous_rotation;
    bool zero_crossed = false;
    bool target_reached = false;
    bool exit_condition = false;
    float xyz[3];

    // Try to get the current orientation of the robot
    if (!this->bno->getEuler(xyz))
    {
        printf("HardwareInterface::startRotation() -> Failure while getting rotation data...aborting request\n");
        this->rotation_running = false;
        return;
    }
    // Check the desired direction
    switch (request->direction)
    {
        case Rotation::Direction::LEFT:
            // We need to decrease the current orientation
            target_rotation = xyz[0] - request->degree;
            this->rotateLeft();
            break;

        case Rotation::Direction::RIGHT:
            // We need to increase the current orientation
            target_rotation = xyz[0] + request->degree;
            this->rotateRight();
            break;

        // Invalid direction desired
        default:
            printf("HardwareInterface::startRotation() -> "
                   "Unknown direction (%d) request received...dropping request\n", request->direction);
            this->rotation_running = false;
            return;
    }
    previous_rotation = xyz[0];

    // Runs until exit condition is true (means abort or desired rotation reached)
    while (!exit_condition)
    {
        // Wait a specified time for a message
        if (poll(&pfd, 1, Rotation::ROTATION_AGENT_TIMEOUT_MS) == -1)
        {
            fprintf(stderr, "HardwareInterface::startRotation() -> "
                            "Error while poll(): %s...Aborting rotation\n", strerror(errno));
            this->rotation_running = false;
            return;
        }
        // True, if a message came in
        else if (pfd.revents & POLLIN)
        {
            // Update the exit condition variable
            ssize_t n = read(pfd.fd, &exit_condition, sizeof(exit_condition));
            if (n == -1)
            {
                fprintf(stderr, "HardwareInterface::startRotation() -> "
                                "Error while read() on rotation pipe: %s...Aborting rotation\n", strerror(errno));
                this->rotation_running = false;
                return;
            }
            continue;
        }
        // True, if the timer expired
        else
        {
            // Try to get the current orientation
            if (!this->bno->getEuler(xyz))
            {
                printf("HardwareInterface::startRotation() -> "
                       "Failure while getting rotation data...keep on trying\n");
                continue;
            }
            switch (request->direction)
            {
                case Rotation::Direction::LEFT:
                    // True, if we crossed zero degrees in this step
                    if ((previous_rotation < (xyz[0] - 100.)) && !zero_crossed) zero_crossed = true;
                    // True, if we crossed zero degrees
                    if (zero_crossed) xyz[0] -= 360;
                    // Compare the actual and desired rotation
                    if (xyz[0] <= target_rotation) target_reached = true;
                    break;

                case Rotation::Direction::RIGHT:
                    // True, if we crossed zero degrees in this step
                    if ((previous_rotation > (xyz[0] + 100.)) && !zero_crossed) zero_crossed = true;
                    // True, if we crossed zero degrees
                    if (zero_crossed) xyz[0] += 360.;
                    // Compare the actual and desired rotation
                    if (xyz[0] >= target_rotation) target_reached = true;
                    break;
            }
            // Break the loop, if the target orientation is reached
            if (target_reached) break;
        }
    }
    // Stop the motors
    this->stopMotors();
    // Overwrite the pollfd values with the passed file descriptor and an output event
    pfd.fd = response_fd;
    pfd.events = POLLOUT;
    // Lock the write mutex with a scoped lock
    std::lock_guard<std::mutex> lock(this->write_mutex);
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::startRotation() -> "
                        "Error while poll(): %s\n", strerror(errno));
    }
    // Write to the file descriptor
    else if (write(response_fd, &Action::ROTATION_COMPLETE, sizeof(Action::ROTATION_COMPLETE)) == -1)
        fprintf(stderr, "HardwareInterface::startRotation() -> Error while write(): %s\n", strerror(errno));

    this->rotation_running = false;
}

void HardwareInterface::rotateLeft()
{
    // Send commands to all motors to let the robot turn left
    this->motor_fr->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_rr->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_fl->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_rl->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
}

void HardwareInterface::rotateRight()
{
    // Send commands to all motors to let the robot turn right
    this->motor_fr->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_rr->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_fl->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_rl->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
}

void HardwareInterface::stopMotors()
{
    // Stop all motors
    this->motor_fr->stop();
    this->motor_rr->stop();
    this->motor_fl->stop();
    this->motor_rl->stop();
}

bool HardwareInterface::ultraSonicRunning()
{
    // Return whether an ultra sonic measurement is running
    return this->us_running;
}

bool HardwareInterface::rotationMovementRunning()
{
    // Return whether a rotation movement is running
    return this->rotation_running;
}

int HardwareInterface::getRotationPipe()
{
    // Return the writing end of the rotation pipe
    return this->rotation_pipe[1];
}
