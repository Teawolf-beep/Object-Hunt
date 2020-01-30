//
// Created by robert on 11/24/19.
//

#ifndef OBJECT_HUNT_BASE_VMA_401_H
#define OBJECT_HUNT_BASE_VMA_401_H

// System includes
#include <cstdint>
#include <memory>
#include <atomic>
#include <thread>

// Project includes
#include "VMA_401Exception.h"

// Forward declarations
class GPIO;

/**
 * A class for the VMA 401 stepper motor.
 * Helps with setting up and moving the stepper motor.
 */
class VMA_401
{
public:

    /**
     * Custom constructor.
     * Initializes the passed pins.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param pin1 Pin 1 of the stepper motor. Please see the Stepper::Pin namespace for further information.
     * @param pin2 Pin 2 of the stepper motor. Please see the Stepper::Pin namespace for further information.
     * @param pin3 Pin 3 of the stepper motor. Please see the Stepper::Pin namespace for further information.
     * @param pin4 Pin 4 of the stepper motor. Please see the Stepper::Pin namespace for further information.
     * @throw VMA_401Exception in case something went wrong.
     */
    VMA_401(std::shared_ptr<GPIO> gpio, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);

    /**
     * Custom destructor.
     * Frees hardware resources and joins the child thread.
     */
    ~VMA_401();

    /**
     * Starts a movement of the stepper motor.
     * The movement will be conducted by a dedicated thread. This means, that the calling thread will return from this
     * function almost immediately. The stepper motor turns until he moved the desired steps or this method is called again.
     *
     * @param direction The direction of the movement. Please see the namespace Stepper::Direction for possible values.
     * @param steps The number of steps. The stepper motor will move only the specified count of steps.
     * @param idle_time Idle time between steps. With this parameter the speed of the movement can be controlled.
     */
    void startMoving(uint8_t direction, uint16_t steps, uint16_t idle_time);

    /**
     * Stops the actual movement immediately.
     * Aborts the current movement and joins the moving thread. If the stepper motor is not turning, nothing happens.
     */
    void stop();

private:
    void terminateThread();
    void move(uint8_t dir, uint16_t steps, uint16_t idle_time);
    void set_output(uint8_t out, bool &exit_condition, std::chrono::microseconds &timeout);

    std::shared_ptr<GPIO> gpio;
    uint8_t pin_1;
    uint8_t pin_2;
    uint8_t pin_3;
    uint8_t pin_4;
    std::atomic<bool> thread_running;
    std::thread mover;
    int pipe_fd[2];
};

#endif //OBJECT_HUNT_BASE_VMA_401_H
