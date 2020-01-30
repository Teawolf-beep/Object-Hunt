//
// Created by robert on 09.01.20.
//

#ifndef OBJECT_HUNT_BASE_HC_SR04EXCEPTION_H
#define OBJECT_HUNT_BASE_HC_SR04EXCEPTION_H

/**
 * Exception thrown by the HC_SR04 (ultra sonic sensor) class.
 */
class HC_SR04Exception : public std::exception
{
protected:
    std::string message; /**< The error message.*/

public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit HC_SR04Exception(const std::string& message) : message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~HC_SR04Exception() throw() {}

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
#endif //OBJECT_HUNT_BASE_HC_SR04EXCEPTION_H
