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
    std::vector<InterruptDescriptor> interrupts;

    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::FRONT_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::REAR_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::LEFT_ECHO, Edge::BOTH});
    interrupts.push_back(InterruptDescriptor {UltraSonic::Pin::RIGHT_ECHO, Edge::BOTH});

    interrupts.push_back(InterruptDescriptor {Revolution::Pin::FRONT_RIGHT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::FRONT_LEFT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::REAR_LEFT, Edge::FALLING});
    interrupts.push_back(InterruptDescriptor {Revolution::Pin::REAR_RIGHT, Edge::FALLING});

    if (!this->gpio->setMultipleKernelDriver(interrupts, PullState::OFF))
        throw HardwareInterfaceException("HardwareInterface() -> Error while setting GPIO kernel driver!");

    this->us_front = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::FRONT, UltraSonic::Pin::FRONT_TRIGGER,
                                               UltraSonic::Pin::FRONT_ECHO,pollfd {interrupts[0].fd, POLLPRI});
    this->us_rear = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::REAR, UltraSonic::Pin::REAR_TRIGGER,
                                              UltraSonic::Pin::REAR_ECHO, pollfd {interrupts[1].fd, POLLPRI});
    this->us_left = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::LEFT, UltraSonic::Pin::LEFT_TRIGGER,
                                              UltraSonic::Pin::LEFT_ECHO,pollfd {interrupts[2].fd, POLLPRI});
    this->us_right = std::make_unique<HC_SR04>(gpio, UltraSonic::Device::RIGHT, UltraSonic::Pin::RIGHT_TRIGGER,
                                               UltraSonic::Pin::RIGHT_ECHO, pollfd {interrupts[3].fd, POLLPRI});

    this->rev_fr = std::make_unique<TCST_1103>(gpio, Revolution::Pin::FRONT_RIGHT, interrupts[4].fd);
    this->rev_fl = std::make_unique<TCST_1103>(gpio, Revolution::Pin::FRONT_LEFT, interrupts[5].fd);
    this->rev_rr = std::make_unique<TCST_1103>(gpio, Revolution::Pin::REAR_LEFT, interrupts[6].fd);
    this->rev_rl = std::make_unique<TCST_1103>(gpio, Revolution::Pin::REAR_RIGHT, interrupts[7].fd);

    this->gpio->makeOutput(Motor::ENABLE_ALL);

    bno->setOperationMode(BNO055::OperationMode::IMUPLUS);

    this->speed_agent = std::thread(&HardwareInterface::updateSpeed, this);
    this->enableMotors();
}

HardwareInterface::~HardwareInterface()
{
    this->stopMotors();
    this->disableMotors();
    this->terminateSpeedAgent();
    if (this->us_agent.joinable()) this->us_agent.join();
//    printf("~HardwareInterface() -> Ultra sonic is %s running\n", (this->us_running) ? "" : "not");
//    printf("~HardwareInterface() GPIO count: %ld\n", this->gpio.use_count());
}

void HardwareInterface::enableMotors()
{
    this->gpio->setPin(Motor::ENABLE_ALL);
}

void HardwareInterface::disableMotors()
{
    this->gpio->clearPin(Motor::ENABLE_ALL);
}

bool HardwareInterface::areMotorsEnabled()
{
    return this->gpio->readPin(Motor::ENABLE_ALL);
}

bool HardwareInterface::moveMotor(struct Motor::Command *command)
{
    if ((command->direction != Motor::Direction::FORWARD) && (command->direction != Motor::Direction::BACKWARD))
    {
        printf("HardwareInterface::moveMotor() -> "
               "Unknown direction command (%d) received"
               "...ignoring command.\n", command->direction);
        return false;
    }
    if (command->devices & Motor::Device::FRONT_RIGHT)
    {
        if (command->duty_cycle == 0) this->motor_fr->stop();
        else this->motor_fr->startMoving(command->direction, command->duty_cycle);
    }
    if (command->devices & Motor::Device::FRONT_LEFT)
    {
        if (command->duty_cycle == 0) this->motor_fl->stop();
        else this->motor_fl->startMoving(command->direction, command->duty_cycle);
    }
    if (command->devices & Motor::Device::BACK_RIGHT)
    {
        if (command->duty_cycle == 0) this->motor_rr->stop();
        else this->motor_rr->startMoving(command->direction, command->duty_cycle);
    }
    if (command->devices & Motor::Device::BACK_LEFT)
    {
        if (command->duty_cycle == 0) this->motor_rl->stop();
        else this->motor_rl->startMoving(command->direction, command->duty_cycle);
    }
    return true;
}

bool HardwareInterface::moveStepper(struct Stepper::Command *command)
{
    if (command->steps > Stepper::MAX_STEPS)
    {
        printf("HardwareInterface::moveStepper() -> "
               "Step number is higher than maximum permitted number"
               "...ignoring command.\n");
        return false;
    }
    else if ((command->direction != Stepper::Direction::RIGHT) && (command->direction != Stepper::Direction::LEFT))
    {
        printf("HardwareInterface::moveStepper() -> "
               "Unknown direction command (%d)"
               "...ignoring command.\n", command->direction);
        return false;
    }
    else if (command->steps == 0) this->stepper->stop();
    else this->stepper->startMoving(command->direction, command->steps, Stepper::IDLE_TIME);
    return true;
}

void HardwareInterface::startUltraSonic(int response_fd, struct UltraSonic::Request *request)
{
    if (this->us_running)
    {
        struct UltraSonic::Response response = {request->id, -1.,
                                                -1., -1., -1.};
        struct pollfd pfd_response = {response_fd, POLLOUT};

        std::lock_guard<std::mutex> lock(this->write_mutex);
        if (poll(&pfd_response, 1, -1) == -1)
            fprintf(stderr, "HardwareInterface::startUltraSonic() -> "
                            "Error while poll() on navigation fd: %s\n", strerror(errno));
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
        if (this->us_agent.joinable()) this->us_agent.join();
        this->us_agent = std::thread(&HardwareInterface::measureUltraSonic,
                                     this, response_fd, request);
    }
}

void HardwareInterface::measureUltraSonic(int response_fd, struct UltraSonic::Request* request)
{
    this->us_running = true;

    uint8_t measurements = 0;
    ssize_t n;
    struct pollfd pfd = {response_fd, POLLOUT};
    struct UltraSonic::Response response = {request->id, -1., -1., -1., -1.};
    struct InternalUSResponse internal_response;

    if (request->devices & UltraSonic::Device::FRONT)
    {
        this->us_front->measure(this->ultra_sonic_pipe[1]);
        ++measurements;
    }
    if (request->devices & UltraSonic::Device::REAR)
    {
        this->us_rear->measure(this->ultra_sonic_pipe[1]);
        ++measurements;
    }
    if (request->devices & UltraSonic::Device::LEFT)
    {
        this->us_left->measure(this->ultra_sonic_pipe[1]);
        ++measurements;
    }
    if (request->devices & UltraSonic::Device::RIGHT)
    {
        this->us_right->measure(this->ultra_sonic_pipe[1]);
        ++measurements;
    }
    if (measurements < 1) printf("HardwareInterface::measureUltraSonic() -> "
                                 "Invalid device sequence (%02X)...Ignoring request", request->devices);
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
//            printf("Response received! Device: %d; Distance: %f\n", internal_response.device, internal_response.distance);
    }
    std::lock_guard<std::mutex> lock(this->write_mutex);
    if (poll(&pfd, 1, -1) == -1)
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while poll(): %s\n", strerror(errno));
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
    struct pollfd pfd = {response_fd, POLLOUT};
    struct Revolution::Response response = {request->id, -1, -1,
            -1, -1};

    if (request->devices & Revolution::Device::FRONT_RIGHT) response.count_front_right = this->rev_fr->getCounter();
    if (request->devices & Revolution::Device::FRONT_LEFT) response.count_front_left = this->rev_fl->getCounter();
    if (request->devices & Revolution::Device::REAR_RIGHT) response.count_rear_right = this->rev_rr->getCounter();
    if (request->devices & Revolution::Device::REAR_LEFT) response.count_rear_left = this->rev_rl->getCounter();
    std::lock_guard<std::mutex> lock(this->write_mutex);
    if (poll(&pfd, 1, -1) == -1)
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while poll(): %s\n", strerror(errno));

    else if ((write(response_fd, &Action::REVOLUTION, sizeof(Action::REVOLUTION)) == -1) ||
             (write(response_fd, &response, sizeof(response)) == -1))
    {
        fprintf(stderr, "HardwareInterface::measureUltraSonic() -> "
                        "Error while write(): %s\n", strerror(errno));
    }
}

bool HardwareInterface::getSpeed(int response_fd, uint32_t id)
{
    struct pollfd pfd = {response_fd, POLLOUT};
    struct Speed::Response response = {id, this->speed};

    std::lock_guard<std::mutex> lock(this->write_mutex);
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::getSpeed() -> "
                        "Error while poll(): %s\n", strerror(errno));
        return false;
    }
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
    struct pollfd pfd = {response_fd, POLLOUT};
    struct Orientation::Response response = {id, 1000., 1000., 1000.};
    bool ret = false;

    if (this->bno->getEuler(response.xyz)) ret = true;

    std::lock_guard<std::mutex> lock(this->write_mutex);
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::getOrientation() -> "
                        "Error while poll(): %s\n", strerror(errno));
        ret = false;
    }
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
    struct pollfd pfd = {this->speed_pipe[0], POLLIN};
    std::chrono::steady_clock::time_point time_reference;
    float prev_rev_count, current_rev_count = 0, change_rev_count;
    uint8_t current_direction;

    while (!exit_condition)
    {
        prev_rev_count = current_rev_count;
        current_rev_count = this->getRevolutionMedian();
        current_direction = this->motor_fl->getDirection() & this->motor_fr->getDirection() &
                            this->motor_rl->getDirection() & this->motor_rr->getDirection();

        switch (current_direction)
        {
            case Motor::Direction::FORWARD:
                change_rev_count = current_rev_count - prev_rev_count;
                break;

            case Motor::Direction::BACKWARD:
                change_rev_count = prev_rev_count - current_rev_count;
                break;

            default:
                change_rev_count = 0;
        }
        this->speed = change_rev_count * Speed::DISTANCE_PER_COUNT_CM /
                       std::chrono::duration<float>(std::chrono::steady_clock::now() - time_reference).count();


//        buffer = this->value;
////        printf("motor_fl: %d, motor_fr: %d, motor_rl: %d, motor_rr: %d\n", this->motor_fl->getDirection(), this->motor_fr->getDirection(), this->motor_rl->getDirection(), this->motor_rr->getDirection());
//        printf("current_direction: %d, value: %f\n", current_direction, buffer);

        time_reference = std::chrono::steady_clock::now();

        if (poll(&pfd, 1, Speed::MEASUREMENT_INTERVAL_MS) == -1)
        {
            fprintf(stderr, "HardwareInterface::updateSpeed() -> "
                            "Error while poll(): %s\n", strerror(errno));
            return;
        }
        else if (pfd.revents & POLLIN)
        {
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
    return float((this->rev_fr->getCounter() + this->rev_fl->getCounter() +
                  this->rev_rr->getCounter() + this->rev_rl->getCounter())/4.);
}

void HardwareInterface::terminateSpeedAgent()
{
    bool exit_condition = true;
    write(this->speed_pipe[1], &exit_condition, sizeof(bool));
    this->speed_agent.join();
}

void HardwareInterface::startRotation(int response_fd, struct Rotation::Command *request)
{
    this->rotation_running = true;

    struct pollfd pfd = {this->rotation_pipe[0], POLLIN};
    float target_rotation;
    float previous_rotation;
    bool zero_crossed = false;
    bool target_reached = false;
    bool exit_condition = false;
    float xyz[3];

    if (!this->bno->getEuler(xyz))
    {
        printf("HardwareInterface::startRotation() -> Failure while getting rotation data...aborting request\n");
        this->rotation_running = false;
        return;
    }
    switch (request->direction)
    {
        case Rotation::Direction::LEFT:
            target_rotation = xyz[0] - request->degree;
            this->rotateLeft();
            break;

        case Rotation::Direction::RIGHT:
            target_rotation = xyz[0] + request->degree;
            this->rotateRight();
            break;

        default:
            printf("HardwareInterface::startRotation() -> "
                   "Unknown direction (%d) request received...dropping request\n", request->direction);
            this->rotation_running = false;
            return;
    }
    previous_rotation = xyz[0];

    while (!exit_condition)
    {
        if (poll(&pfd, 1, Rotation::ROTATION_AGENT_TIMEOUT_MS) == -1)
        {
            fprintf(stderr, "HardwareInterface::startRotation() -> "
                            "Error while poll(): %s...Aborting rotation\n", strerror(errno));
            this->rotation_running = false;
            return;
        }
        else if (pfd.revents & POLLIN)
        {
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
        else
        {
            if (!this->bno->getEuler(xyz))
            {
                printf("HardwareInterface::startRotation() -> "
                       "Failure while getting rotation data...keep on trying\n");
                continue;
            }
            switch (request->direction)
            {
                case Rotation::Direction::LEFT:
                    if ((previous_rotation < (xyz[0] - 100.)) && !zero_crossed) zero_crossed = true;
                    if (zero_crossed) xyz[0] -= 360;
                    if (xyz[0] <= target_rotation) target_reached = true;
                    break;

                case Rotation::Direction::RIGHT:
                    if ((previous_rotation > (xyz[0] + 100.)) && !zero_crossed) zero_crossed = true;
                    if (zero_crossed) xyz[0] += 360.;
                    if (xyz[0] >= target_rotation) target_reached = true;
                    break;
            }
            if (target_reached) break;
        }
    }
    this->stopMotors();

    pfd.fd = response_fd;
    pfd.events = POLLOUT;

    std::lock_guard<std::mutex> lock(this->write_mutex);
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr, "HardwareInterface::startRotation() -> "
                        "Error while poll(): %s\n", strerror(errno));
    }
    else if (write(response_fd, &Action::ROTATION_COMPLETE, sizeof(Action::ROTATION_COMPLETE)) == -1)
        fprintf(stderr, "HardwareInterface::startRotation() -> Error while write(): %s\n", strerror(errno));

    this->rotation_running = false;
}

void HardwareInterface::rotateLeft()
{
    this->motor_fr->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_rr->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_fl->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_rl->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
}

void HardwareInterface::rotateRight()
{
    this->motor_fr->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_rr->startMoving(Motor::Direction::BACKWARD, Rotation::DUTY_CYCLE);
    this->motor_fl->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
    this->motor_rl->startMoving(Motor::Direction::FORWARD, Rotation::DUTY_CYCLE);
}

void HardwareInterface::stopMotors()
{
    this->motor_fr->stop();
    this->motor_rr->stop();
    this->motor_fl->stop();
    this->motor_rl->stop();
}

bool HardwareInterface::ultraSonicRunning()
{
    return this->us_running;
}

bool HardwareInterface::rotationMovementRunning()
{
    return this->rotation_running;
}

int HardwareInterface::getRotationPipe()
{
    return this->rotation_pipe[1];
}
