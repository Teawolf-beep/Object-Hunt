//
// Created by robert on 11/24/19.
//

#ifndef OBJECT_HUNT_BASE_GEARMOTOR_H
#define OBJECT_HUNT_BASE_GEARMOTOR_H

// System includes
#include <memory>
#include <thread>
#include <atomic>

// Project includes
#include "GearMotorException.h"

// Forward declarations
class GPIO;

/**
 * A class for a generic gear motor.
 * Helps with setting up the motor and performing movements.
 */
class GearMotor
{
public:
    /**
     * Custom constructor.
     * Initializes the hardware pins and creates the object.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param forward The forward pin. This pin lets the motor driver rotate the pin forward.
     * @param backward The backward pin. This pin lets the motor driver rotate the motor backward.
     * @param frequency he frequency with which the PWM control will work.
     */
    GearMotor(std::shared_ptr<GPIO> gpio, uint8_t forward, uint8_t backward, uint32_t frequency);

    /**
     * Custom destructor.
     * Terminates the child thread and frees hardware resources.
     */
    ~GearMotor();

    /**
     * Starts a movement of the gear motor.
     * The movement will be conducted by a dedicated thread. This means that the calling thread will return from
     * this function almost immediately. The motor runs until he gets a new movement or stop command.
     *
     * @param direction the direction of the movement. Please see Motor::Direction for further information.
     * @param duty_cycle The duty cycle which should be applied to the motor.
     */
    void startMoving(uint8_t direction, uint16_t duty_cycle);

    /**
     * Stops the motor.
     * The moving thread will be stopped and joined immediately.
     */
    void stop();

    /**
     * Gets the current moving direction of the motor.
     *
     * @return The current direction. Please see Motor::Direction for possible values.
     */
    uint8_t getDirection();

private:
    void terminateThread();
    void move(uint8_t pin, uint16_t duty_cycle);

    std::shared_ptr<GPIO> gpio;
    uint8_t pin_forward;
    uint8_t pin_backward;
    std::thread mover;
    int control_pipe[2];
    uint32_t period_duration;
    std::atomic<bool> thread_running;
    uint8_t moving_direction;
};

#endif //OBJECT_HUNT_BASE_GEARMOTOR_H
