//
// Created by robert on 27.11.19.
//

#ifndef OBJECT_HUNT_BASE_HUNTEREXCEPTION_H
#define OBJECT_HUNT_BASE_HUNTEREXCEPTION_H

/**
 * Exception thrown by the HardwareInterface class.
 */
class HardwareInterfaceException : virtual public std::exception
{
public:
    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit HardwareInterfaceException(const std::string& message) : message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~HardwareInterfaceException() throw() {}

    /**
     * Gets the error message.
     *
     * @return Pointer to a null-terminated string with explanatory information.
     */
    virtual const char* what() const throw()
    {
        return this->message.c_str();
    }

protected:
    std::string message; /**< The error message.*/
};

#endif //OBJECT_HUNT_BASE_HUNTEREXCEPTION_H
