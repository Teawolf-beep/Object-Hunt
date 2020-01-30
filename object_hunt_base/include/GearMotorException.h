//
// Created by robert on 11/24/19.
//

#ifndef OBJECT_HUNT_BASE_GEARMOTOREXCEPTION_H
#define OBJECT_HUNT_BASE_GEARMOTOREXCEPTION_H

/**
 * Exception thrown by the GearMotor class.
 */
class GearMotorException : public std::exception
{
protected:
    std::string message; /**< The error message.*/

public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit GearMotorException(const std::string& message):
            message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~GearMotorException() throw() {}

    /**
     * Gets the error message.
     *
     * @return Pointer to a null-terminated string with explanatory information.
     */
    virtual const char* what() const throw()
    {
        return this->message.c_str();
    }
};

#endif //OBJECT_HUNT_BASE_GEARMOTOREXCEPTION_H
