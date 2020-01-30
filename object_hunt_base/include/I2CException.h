//
// Created by robert on 05.12.19.
//

#ifndef OBJECT_HUNT_BASE_I2CEXCEPTION_H
#define OBJECT_HUNT_BASE_I2CEXCEPTION_H

/**
 * Exception thrown by the I2C class.
 */
class I2CException : public std::exception
{
protected:
    std::string message; /**< The error message.*/

public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit I2CException(const std::string& message) :
            message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~I2CException() throw() {}

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

#endif //OBJECT_HUNT_BASE_I2CEXCEPTION_H
