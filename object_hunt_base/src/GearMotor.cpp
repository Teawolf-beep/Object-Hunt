//
// Created by robert on 11/24/19.
//

// System includes
#include <iostream>
#include <cstring>
#include <poll.h>
#include <unistd.h>

// Project includes
#include "GearMotor.h"
#include "GPIO.h"
#include "ObjectHuntDefinitions.h"

GearMotor::GearMotor(std::shared_ptr<GPIO> gpio, uint8_t forward, uint8_t backward, uint32_t frequency)
        : gpio(gpio),
          pin_forward(forward),
          pin_backward(backward),
          thread_running(false),
          period_duration(1000000/frequency),
          moving_direction(Motor::Direction::NOT_MOVING)
{
    if ((frequency < 1) || (frequency > 10000))
        throw GearMotorException("GearMotor() -> "
                             "Frequency has to be between 1 Hz and 10000 Hz!");

    if (pipe(this->control_pipe))
        throw GearMotorException("GearMotor() -> Error while creating internal pipe: " +
                                 std::string(std::strerror(errno)));

    this->gpio->makeOutput(this->pin_forward);
    this->gpio->makeOutput(this->pin_backward);
}

GearMotor::~GearMotor()
{
    if (this->thread_running) this->terminateThread();
//    printf("~GearMotor() GPIO count: %ld\n", this->gpio.use_count());
    this->gpio->releaseOutput(this->pin_forward);
    this->gpio->releaseOutput(this->pin_backward);
}

void GearMotor::stop()
{
    if (this->thread_running) this->terminateThread();
    this->thread_running = false;
    this->moving_direction = Motor::Direction::NOT_MOVING;
}

void GearMotor::terminateThread()
{
    bool exit_condition = true;
    write(this->control_pipe[1], &exit_condition, sizeof(bool));
    this->mover.join();
}

uint8_t GearMotor::getDirection()
{
    return this->moving_direction;
}

void GearMotor::startMoving(uint8_t direction, uint16_t duty_cycle)
{
    if (duty_cycle > 100) duty_cycle = 100;
    if (this->thread_running) this->terminateThread();
    this->mover = std::thread(&GearMotor::move, this,
            (direction == Motor::Direction::FORWARD) ? this->pin_forward : this->pin_backward, duty_cycle);
    this->moving_direction = direction;
}

void GearMotor::move(uint8_t pin, uint16_t duty_cycle)
{
    this->thread_running = true;
    std::chrono::microseconds timeout_high((long) ((float) this->period_duration/100) * duty_cycle);
    std::chrono::microseconds timeout_low((long) this->period_duration - timeout_high.count());
    struct pollfd pfd = {this->control_pipe[0], POLLIN};
    bool exit_condition = false;

    while (!exit_condition)
    {
        this->gpio->setPin(pin);
        std::this_thread::sleep_for(timeout_high);
        if (poll(&pfd, 1, 0) == -1)
        {
            fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                            "Error while poll(): %s\n", this->pin_forward, strerror(errno));
            break;
        }
        else if (pfd.revents & POLLIN)
        {
            ssize_t n = read(this->control_pipe[0], &exit_condition, sizeof(bool));
            if (n == -1)
            {
                fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                                "Error while read(): %s\n", this->pin_forward, strerror(errno));
                break;
            }
            continue;
        }
        this->gpio->clearPin(pin);
        std::this_thread::sleep_for(timeout_low);
        if (poll(&pfd, 1, 0) == -1)
        {
            fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                            "Error while poll(): %s\n", this->pin_forward, strerror(errno));
            break;
        }
        else if (pfd.revents & POLLIN)
        {
            ssize_t n = read(this->control_pipe[0], &exit_condition, sizeof(bool));
            if (n == -1)
            {
                fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                                "Error while read(): %s\n", this->pin_forward, strerror(errno));
                perror("");
                break;
            }
        }
    }
    this->gpio->clearPin(pin);
    this->thread_running = false;
}
