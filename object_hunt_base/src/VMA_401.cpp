//
// Created by robert on 11/24/19.
//

// System includes
#include <cstring>
#include <chrono>
#include <unistd.h>
#include <sys/poll.h>

// Project includes
#include "VMA_401.h"
#include "ObjectHuntDefinitions.h"
#include "GPIO.h"

// Constructor. Initializes members via member initializer list.
VMA_401::VMA_401(std::shared_ptr<GPIO> gpio, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
        : gpio(gpio), pin_1(pin1), pin_2(pin2), pin_3(pin3), pin_4(pin4), thread_running(false)
{
    // Try to create an internal control pipe. True, if an error occurred
    if (pipe(this->pipe_fd))
        throw VMA_401Exception("VMA_401() -> Error while creating the pipe " +
                               std::string(std::strerror(errno)));
    // Make all passed pins to outputs
    this->gpio->makeOutput(this->pin_1);
    this->gpio->makeOutput(this->pin_2);
    this->gpio->makeOutput(this->pin_3);
    this->gpio->makeOutput(this->pin_4);
}

VMA_401::~VMA_401()
{
    // Terminate the moving thread (if it is running)
    this->terminateThread();
    // Release all outputs
    this->gpio->releaseOutput(this->pin_1);
    this->gpio->releaseOutput(this->pin_2);
    this->gpio->releaseOutput(this->pin_3);
    this->gpio->releaseOutput(this->pin_4);
}

void VMA_401::stop()
{
    // Terminate the moving thread (if it is running)
    this->terminateThread();
    this->thread_running = false;
}

void VMA_401::terminateThread()
{
    // Check, if the mover thread is actually running
    if (this->thread_running && this->mover.joinable())
    {
        // Create an exit condition variable with value true
        bool exit_condition = true;
        // Write the exit condition variable to the internal pipe
        write(this->pipe_fd[1], &exit_condition, sizeof(bool));
        // Wait for the rotation thread to terminate and join it
        this->mover.join();
    }
    // Check, if the mover thread can be joined
    else if (this->mover.joinable()) this->mover.join();
}

void VMA_401::startMoving(uint8_t direction, uint16_t steps, uint16_t idle_time)
{
    // Terminate the moving thread (if it is running)
    this->terminateThread();
    this->thread_running = true;
    // Start a movement with the passed parameters
    this->mover = std::thread(&VMA_401::move, this, direction, steps, idle_time);
}

void VMA_401::move(uint8_t dir, uint16_t steps, uint16_t idle_time)
{
    std::chrono::microseconds timeout(idle_time);
    uint16_t counter = 0;
    bool exit_condition = false;

    switch (dir)
    {
        case Stepper::Direction::LEFT:
            // Move until the desired steps are reached
            while ((counter < steps) && !exit_condition)
            {
                for(int16_t i = 7; i >= 0; --i)
                {
                    // Move the motor and wait the passed timeout
                    this->set_output(i, exit_condition, timeout);
                    // Check, if a termination signal was received
                    if (exit_condition) break;
                }
                // Increment the step counter
                ++counter;
            }
            break;

        case Stepper::Direction::RIGHT:
            // Move until the desired steps are reached
            while ((counter < steps) && !exit_condition)
            {
                for(int16_t i = 0; i < 8; ++i)
                {
                    // Move the motor and wait the passed timeout
                    this->set_output(i, exit_condition, timeout);
                    // Check, if a termination signal was received
                    if (exit_condition) break;
                }
                // Increment the step counter
                ++counter;
            }
            break;

        default:
            break;
    }
    this->thread_running = false;
}

void VMA_401::set_output(uint8_t out, bool &exit_condition, std::chrono::microseconds &timeout)
{
    // Initialize a pollfd with the internal pipe and an input event
    static struct pollfd pfd = {this->pipe_fd[0], POLLIN};

    // Set the output respecting to current state and lookup table
    (Stepper::LOOKUP[out] & 1) ? this->gpio->setPin(this->pin_1) : this->gpio->clearPin(this->pin_1);
    (Stepper::LOOKUP[out] & 2) ? this->gpio->setPin(this->pin_2) : this->gpio->clearPin(this->pin_2);
    (Stepper::LOOKUP[out] & 4) ? this->gpio->setPin(this->pin_3) : this->gpio->clearPin(this->pin_3);
    (Stepper::LOOKUP[out] & 8) ? this->gpio->setPin(this->pin_4) : this->gpio->clearPin(this->pin_4);
    // Sleep the passed timeout
    std::this_thread::sleep_for(timeout);
    // Check, if a message was sent in the internal pipe
    if (poll(&pfd, 1, 0) == -1) fprintf(stderr, "VMA_401::move() -> "
                        "Error while poll(): %s\n", strerror(errno));
    // True, if a message was received
    else if (pfd.revents & POLLIN)
    {
        // Update the exit condition
        ssize_t n = read(this->pipe_fd[0], &exit_condition, sizeof(bool));
        if (n == -1) fprintf(stderr, "VMA_401::move() -> "
                            "Error while read(): %s\n", strerror(errno));
    }
}
