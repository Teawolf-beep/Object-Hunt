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

// Constructor. Initializes members via member initializer list.
GearMotor::GearMotor(std::shared_ptr<GPIO> gpio, uint8_t forward, uint8_t backward, uint32_t frequency)
        : gpio(gpio),
          pin_forward(forward),
          pin_backward(backward),
          thread_running(false),
          period_duration(1000000/frequency),
          moving_direction(Motor::Direction::NOT_MOVING)
{
    // True, if the passed frequency is out of range
    if ((frequency < 1) || (frequency > 10000))
        throw GearMotorException("GearMotor() -> "
                             "Frequency has to be between 1 Hz and 10000 Hz!");
    // True, if an error occurred while creating a control pipe
    if (pipe(this->control_pipe))
        throw GearMotorException("GearMotor() -> Error while creating internal pipe: " +
                                 std::string(std::strerror(errno)));
    // Make the passed pins to outputs
    this->gpio->makeOutput(this->pin_forward);
    this->gpio->makeOutput(this->pin_backward);
}

GearMotor::~GearMotor()
{
    // Terminate the mover thread, if it is running
    if (this->thread_running) this->terminateThread();
    // Release gpio pins
    this->gpio->releaseOutput(this->pin_forward);
    this->gpio->releaseOutput(this->pin_backward);
}

void GearMotor::stop()
{
    // Terminate the mover thread, if it is running
    if (this->thread_running) this->terminateThread();
    this->thread_running = false;
    // Set the current movement direction
    this->moving_direction = Motor::Direction::NOT_MOVING;
}

void GearMotor::terminateThread()
{
    // Create an exit condition variable with value true
    bool exit_condition = true;
    // Write the variable to the internal control pipe
    write(this->control_pipe[1], &exit_condition, sizeof(bool));
    // Wait for the thread to terminate and join it
    this->mover.join();
}

uint8_t GearMotor::getDirection()
{
    // return the current direction
    return this->moving_direction;
}

void GearMotor::startMoving(uint8_t direction, uint16_t duty_cycle)
{
    // Limit the duty cycle to 100
    if (duty_cycle > 100) duty_cycle = 100;
    // Terminate the mover thread, if it is running
    if (this->thread_running) this->terminateThread();
    // Activate the mover thread with the passed moving options (direction and duty cycle)
    this->mover = std::thread(&GearMotor::move, this,
            (direction == Motor::Direction::FORWARD) ? this->pin_forward : this->pin_backward, duty_cycle);
    // Update the current direction
    this->moving_direction = direction;
}

void GearMotor::move(uint8_t pin, uint16_t duty_cycle)
{
    this->thread_running = true;
    // Calculate the timeouts for the duty cycle
    std::chrono::microseconds timeout_high((long) ((float) this->period_duration/100) * duty_cycle);
    std::chrono::microseconds timeout_low((long) this->period_duration - timeout_high.count());
    // Initialize a pollfd with the internal pipe and an input event
    struct pollfd pfd = {this->control_pipe[0], POLLIN};
    bool exit_condition = false;

    // Execute as long as we are not getting stopped
    while (!exit_condition)
    {
        // Set the passed pin high
        this->gpio->setPin(pin);
        // Sleep for the on duty cycle time
        std::this_thread::sleep_for(timeout_high);
        // Check for any messages
        if (poll(&pfd, 1, 0) == -1)
        {
            fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                            "Error while poll(): %s\n", this->pin_forward, strerror(errno));
            break;
        }
        // True, if we received a message
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
        // Clear the passed pin (set it low)
        this->gpio->clearPin(pin);
        // Sleep for the off duty cycle time
        std::this_thread::sleep_for(timeout_low);
        // Check for any messages
        if (poll(&pfd, 1, 0) == -1)
        {
            fprintf(stderr, "GearMotor::move(), pin forward: %d -> "
                            "Error while poll(): %s\n", this->pin_forward, strerror(errno));
            break;
        }
            // True, if we received a message
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
    // Clear the pi to stop movement
    this->gpio->clearPin(pin);
    this->thread_running = false;
}
