//
// Created by robert on 10.12.19.
//

#ifndef OBJECT_HUNT_NAVIGATION_NAVIGATOREXCEPTION_H
#define OBJECT_HUNT_NAVIGATION_NAVIGATOREXCEPTION_H

/**
 * Exception thrown by the Navigator class.
 */
class NavigatorException : virtual public std::exception
{
public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit NavigatorException(const std::string& message) : message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~NavigatorException() throw() {}

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


#endif //OBJECT_HUNT_NAVIGATION_NAVIGATOREXCEPTION_H
