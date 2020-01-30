//
// Created by robert on 11/23/19.
//

#ifndef OBJECT_HUNT_BASE_TCST_1103EXCEPTION_H
#define OBJECT_HUNT_BASE_TCST_1103EXCEPTION_H

/**
 * Exception thrown by the TCST_1103 class.
 */
class TCST_1103Exception : public std::exception
{
protected:
    std::string message;

public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit TCST_1103Exception(const std::string& message) :
            message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~TCST_1103Exception() throw() {}

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

#endif //OBJECT_HUNT_BASE_TCST_1103EXCEPTION_H
