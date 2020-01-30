//
// Created by robert on 08.01.20.
//

#ifndef OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGEREXCEPTION_H
#define OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGEREXCEPTION_H

#include <string>

/**
 * Exception thrown by the ConnectionManager class.
 */
class ConnectionManagerException : virtual public std::exception
{
public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit ConnectionManagerException(const std::string& message) : message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~ConnectionManagerException() throw() {}

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

#endif //OBJECT_HUNT_NAVIGATION_CONNECTIONMANAGEREXCEPTION_H
