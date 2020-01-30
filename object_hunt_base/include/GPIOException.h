//
// Created by robert on 11/28/19.
//

#ifndef OBJECT_HUNT_BASE_GPIOEXCEPTION_H
#define OBJECT_HUNT_BASE_GPIOEXCEPTION_H

/**
 * Exception thrown by the GPIO class.
 */
class GPIOException : public std::exception
{
protected:
    uint8_t pin; /**< The pin number that caused the error. Objective parameter.*/
    std::string message; /**< The error message.*/

public:
    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     * @param pin The pin number that caused the error. Objective parameter.
     */
    explicit GPIOException(const std::string& message, uint8_t pin=255) :
            pin(pin),
            message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~GPIOException() throw() {}

    /**
     * Gets the error message.
     *
     * @return Pointer to a null-terminated string with explanatory information.
     */
    virtual const char* what() const throw()
    {
        return this->message.c_str();
    }

    /**
     * Gets the pin that caused the error.
     *
     * @return The pin number (BCM numbering!).
     */
    virtual uint8_t getPin() const throw()
    {
        return this->pin;
    }

};

#endif //OBJECT_HUNT_BASE_GPIOEXCEPTION_H
