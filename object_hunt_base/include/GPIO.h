
//
// Created by robert on 11/28/19.
//

#ifndef OBJECT_HUNT_BASE_GPIO_H
#define OBJECT_HUNT_BASE_GPIO_H

#include <vector>
#include <string>
#include <cstdint>

// Project includes
#include "GPIOException.h"

const std::string GPIO_FILENAME = "/dev/gpio"; /**< General GPIO filename. The process will need root permission in order to access this file.*/
const std::string GPIO_USER_FILENAME = "/dev/gpiomem"; /**< User GPIO filename. Available in later Raspbian distributions. Can be used without root permission.*/

/**
 * Possible internal pull states for inputs.
 * Maps each possible pull state to an unsigned number. Please use only these values in order to set the pull state.
 */
namespace PullState
{
    const uint8_t OFF = 0; /**< Internal pull state off.*/
    const uint8_t DOWN = 1; /**< Internal pull state down.*/
    const uint8_t UP = 2; /**< Internal pull state up.*/
}

/**
 * Defines the possible edge types for interrupts.
 * Please pass only these values in order to set up an interrupt kernel driver. Other values will lead to malfunction.
 */
namespace Edge
{
    const std::string FALLING = "falling"; /**< Edge type "falling" */
    const std::string RISING = "rising"; /**< Edge type "rising" */
    const std::string BOTH = "both"; /**< Edge type "both" */
}

/**
 * A simple structure for interrupt description.
 * A reference of this structure needs to be passed in order to implement multiple interrupts at once.
 */
struct InterruptDescriptor
{
    uint8_t pin;  /**< The GPIO pin number connected to the interrupt (BCM numbering scheme!). */
    std::string edge; /**< The edge type. Please see the Edge namespace for further information on possible values. */
    int fd; /**< The file descriptor to listen for the interrupt. Gets an value by a setup function */
};

/**
 * Provides methods to control the RaspberryPi GPIOs.
 * This class implements low level interaction with the RaspberryPi GPIOs. Please take care of race conditions,
 * if used by multiple processes or threads. Tested with RaspberryPi 3 and newer models.
 */
class GPIO {
public:
    /**
     * Custom Constructor.
     * Tries to access and map the GPIO memory of the RaspberryPi.
     *
     * @throw GPIOException in case an error occurred.
     */
    GPIO(const std::string& path);

    /**
     * Custom destructor.
     * Cleans up. Releases all outputs and kernel driver.
     */
    ~GPIO();

    /**
     * Sets a specific pin as an input.
     *
     * @param pin The number of the respecting pin (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool makeInput(uint8_t pin);

    /**
     * Sets a specific pin as an output.
     *
     * @param pin The number of the respecting pin (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool makeOutput(uint8_t pin);

    /**
     * Sets the internal pull state of the passed pin.
     *
     * @param pin The pin (BCM numbering scheme!), whose state shall be changed.
     * @param state The desired state. See namespace PullState for possible values.
     * @return True, if the operation succeeded, false otherwise.
     */
    bool setPullState(uint8_t pin, uint8_t state);

    /**
     * Sets the internal state of the passed pins.
     * Is more time efficient for multiple pins and should be the preferred method for multiple pins.
     * All pins will be changed to the same state. If the
     *
     * @param pins A constant reference to a vector containing the pins (BCM numbering scheme!), whose pull state shall be changed.
     * @param state The desired state for all pins. See namespace PullState for possible values.
     */
    void setPullState(const std::vector<uint8_t>& pins, uint8_t state);

    /**
     * Reads the state of the passed pin.
     *
     * @param pin The regarding pin (BCM numbering scheme!).
     * @return True means high, false means low.
     */
    bool readPin(uint8_t pin);

    /**
     * Sets the passed pin (set to high).
     * The pin needs to be set as an output for the operation to succeed!
     *
     * @param pin The regarding pin (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool setPin(uint8_t pin);

    /**
     * Clears the passed pin (set to low).
     * The pin needs to be set as an output for the operation to succeed!
     *
     * @param pin The regarding GPIO pin (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool clearPin(uint8_t pin);

    /**
     * Creates an interrupt kernel driver for a single pin.
     * Uses sysfs to set up an interrupt kernel driver. There is for the time being no other reliable way to set up pins
     * that are interrupt driven. Assigns a file descriptor to the pin which can be used to read interrupts.
     *
     * @param pin The pin to which the interrupt will be attached.
     * @param pull_state The internal pull state, that will be applied to the interrupt. See namespace PullState for possible values.
     * @return The file descriptor of the regarding interrupt. Positive number if the operation succeeded, negative number in case of an error.
     */
    int setSingleKernelDriver(uint8_t pin, uint8_t pull_state, const std::string& edge);

    /**
     * Creates an interrupt kernel driver for a multiple pins.
     * Does the same as setSingleKernelDriver() but for multiple pins. Is more time efficient than calling setSingleKernelDriver()
     * multiple times and should be the preferred method for multiple pins. The same pull state will be applied to all passed pins.
     *
     * @param descriptors A const reference to a vector of InterruptDescriptor.
     * @param pull_state The internal pull state, that will be applied to all pins. See namespace PullState for possible values.
     * @return True, if the operation succeeded, false otherwise.
     */
    bool setMultipleKernelDriver(std::vector<InterruptDescriptor>& descriptors, uint8_t pull_state);

    /**
     * Releases the associated kernel driver.
     *
     * @param pin The pin, to which the interrupt kernel driver is associated (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool releaseSingleKernelDriver(uint8_t pin);

    /**
     * Releases the passed output.
     * Will release the passed pin by making it an input.The passed pin needs to be set as an output.
     *
     * @param pin The regarding pin (BCM numbering scheme!).
     * @return True, if the operation succeeded, false otherwise.
     */
    bool releaseOutput(uint8_t pin);

    /**
     * Releases all outputs.
     * Will release all known outputs by making them inputs.
     */
    void releaseAllOutputs();

    /**
     * Releases all existing kernel driver.
     */
    void releaseAllKernelDriver();

    /**
     * Checks whether the passed pin is an output or not
     *
     * @param pin The regarding pin (BCM numbering scheme!).
     * @return True, if the passed pin is an output. False otherwise.
     */
    bool isOutput(uint8_t pin);

    /**
     * Checks whether the passed pin has an associated kernel driver.
     *
     * @param pin The regarding pin (BCM numbering scheme!).
     * @return True, if the passed pin has an associated kernel driver. False otherwise.
     */
    bool isKernelDriverSet(uint8_t pin);

    /**
     * Clears all outputs.
     * Clears all known outputs by setting them to low.
     */
    void clearOutputs();

private:
    // Private static methods
    static void writeReg_(volatile uint32_t *addr, uint32_t value);
    static uint32_t readReg_(const volatile uint32_t *addr);
    static void setBits_(volatile uint32_t *addr, uint32_t value);
    void resetPin_(uint8_t  pin);
    void clearPinNoCheck_(uint8_t pin);

    // Private member variables
    // Saves the GPIO base address
    volatile uint32_t *gpio;
    // Saves the set up pin numbers
    std::vector<uint8_t> outputs;
    // Saves the set up kernel numbers
    std::vector<uint8_t> kernel_driver;
};
#endif //OBJECT_HUNT_BASE_GPIO_H
