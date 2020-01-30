//
// Created by robert on 11/22/19.
//

#ifndef OBJECT_HUNT_BASE_TCST_1103_H
#define OBJECT_HUNT_BASE_TCST_1103_H

// System includes
#include <memory>
#include <atomic>
#include <thread>
#include <poll.h>

// Project includes
#include "TCST_1103Exception.h"
#include "GPIOException.h"

// Forward declaration
class GPIO;

/**
 * A class for the TCST 1103 (revolution) sensor.
 * Helps with setting up the sensor and taking measurements.
 */
class TCST_1103
{
public:

    /**
     * Custom constructor.
     * Sets up an interrupt driver for the passed pin on its own. Should be the preferred constructor, if only one
     * revolution sensor is used.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param pin A pin where the revolution signal can be read.
     * @throw TCST_1103Exception in case something went wrong.
     */
    TCST_1103(std::shared_ptr<GPIO> gpio, uint8_t pin);

    /**
     * Custom constructor.
     * Expects an interrupt for the signal pin to be already configured. Saves time, if multiple revolution sensors are used.
     * Should be the preferred constructor, if multiple revolution sensor are used.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param pin A pin where the revolution signal can be read.
     * @param fd A valid file descriptor configured for the signal pin.
     */
    TCST_1103(std::shared_ptr<GPIO> gpio, uint8_t pin, int fd);

    /**
     * Custom destructor.
     * Frees resources and joins the child thread which listens on the signal file descriptor.
     */
    ~TCST_1103();

    /**
     * Gets the current counter value.
     *
     * @return The current counter value.
     */
    uint32_t getCounter();

    /**
     * Resets the counter to zero.
     */
    void resetCounter();

private:
    void listen();

    std::shared_ptr<GPIO> gpio;
    std::thread listener;
    int input_fd;
    int listener_pipe_fd[2];
    uint8_t pin;
    std::atomic<uint32_t> counter;
};

#endif //OBJECT_HUNT_BASE_TCST_1103_H
