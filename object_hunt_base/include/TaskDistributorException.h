//
// Created by robert on 04.12.19.
//

#ifndef OBJECT_HUNT_BASE_TASKDISTRIBUTOREXCEPTION_H
#define OBJECT_HUNT_BASE_TASKDISTRIBUTOREXCEPTION_H

/**
 * Exception thrown by the TaskDistributor class.
 */
class TaskDistributorException : virtual public std::exception
{
public:

    /**
     * Custom constructor.
     *
     * @param message An error message with explanatory information.
     */
    explicit TaskDistributorException(const std::string& message) : message(message)
    {}

    /**
     * Destructor.
     * Virtual to allow inheritance for this class.
     */
    virtual ~TaskDistributorException() throw() {}

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

#endif //OBJECT_HUNT_BASE_TASKDISTRIBUTOREXCEPTION_H
