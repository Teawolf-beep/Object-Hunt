//
// Created by robert on 10.12.19.
//

#ifndef OBJECT_HUNT_NAVIGATION_NAVIGATOR_H
#define OBJECT_HUNT_NAVIGATION_NAVIGATOR_H

#include <cstdint>
#include <memory>

//Project includes
#include "NavigatorException.h"


/**
 * Contains definitions for the Navigator class.
 * Defines threshold values, speed and more.
 */
namespace Navigation
{
    /**
     * Defines timing constants.
     */
    namespace Timeout
    {
        static const int GENERAL_MS = 250; /**< The time in milliseconds the class waits until making new requests.*/
    }

    /**
     * Defines distance thresholds.
     */
    namespace Threshold
    {
        static const float OVERALL_CM = 5.; /**< The overall distance threshold. If an obstacle is nearer than this value,
 * the process will try to do everything to increase the distance.*/
        static const float STRAIGHT_CM = 12.; /**< The distance threshold to an obstacle in front of the car. If an
 * obstacle in front is nearer than this value, the process will try ro change the current direction of movement.*/
        static const float SIDE_CM = 35.; /**< The distance threshold to obstacles on the side. The process considers a gap
 * on the side sufficient big if, the next obstacle is further away than this value.*/
        static const float MOVING_SIDE_MIN_CM = 7.; /**< The distance threshold to obstacles on the side while moving.
 * The process will try to increase distance to the side obstacle, if it is nearer than this value.*/
    }

    /**
     * Defines speed constants.
     */
    namespace Speed
    {
        static const uint8_t STRAIGHT = 70; /**< Duty cycle if the car moves straight.*/
        static const uint8_t SKEW_DIFFERENCE = 40; /**< Duty cycle difference between left and right. Will cause the
 * car to move oblique.*/
    }

    /**
     * Defines rotation constants.
     */
    namespace Rotation
    {
        static const float CHANGE_DIRECTION = 90.; /**< The angle the car will rotate in order to change the moving direction.*/
        static const float AVOID_COLLISION = 20.; /**< The angle the car will rotate in order to avoid a collision.*/
    }

    /**
     * Provides definitions for the state of the Navigator class.
     */
    namespace StateMachine
    {
        /**
         * The possible state of the Navigator class.
         */
        enum States
        {
            STARTING,
            IDLING, /**< Idle state. The process makes no requests but will start/ change navigation depending on incoming events.*/
            MOVING_FORWARD, /**< Moving forward state. The process makes periodically requests to read the attached sensors
 * and steers depending on incoming events.*/
            MOVING_BACKWARD, /**< Moving backward state. The process makes periodically requests to read the attached sensors.
 * The reading of the rear ultra sonic will be given a higher priority.*/
            ROTATING /**< Rotating state. The car is rotating. No requests will be made until the rotation target is reached.*/
        };
    }

    /**
     * Defines orientation constants.
     */
    namespace Orientation
    {
        /**
         * Enumeration to distinguish left and right.
         * Left and right should be seen from the perspective of an imaginary driver of the car.
         */
        enum Side
        {
            Left, /**< The left side.*/
            Right /**< The right side.*/
        };
    }
}
typedef Navigation::StateMachine::States States;
typedef Navigation::Orientation::Side Side;

// Forward declarations
class ConnectionManager;
namespace UltraSonic {struct Response;}
namespace Orientation {struct Response;}
namespace Speed {struct Response;}
namespace Revolution {struct Response;}

/**
 * The application class of this process.
 * This class will be called by the main() function and handles the further program execution.
 */
class Navigator
{
public:

    /**
     * Custom constructor.
     * Prepares inter-thread communication and tries to establish a connection to the base process.
     *
     * @throw NavigatorException in case an error occurred while initialization.
     */
    Navigator();

    /**
     * Custom destructor.
     * Frees resources and closes open ports.
     */
    ~Navigator();

    /**
     * Gets the writing end of the internal control pipe.
     *
     * @return The file descriptor of the writing end.
     */
    int getControlPipe();

    /**
     * The application method.
     * From here the program starts running. Needs to be called after class initialization from the main() function.
     *
     * @return A signed integer number which indicates the termination status of the application.
     */
    int8_t run();

private:
    static void printReadError(const std::string& part);
    static void printIncompleteError(const std::string& part);
    static void printWriteError(const std::string& func, const std::string& part);
    static void shareData(int fd, struct UltraSonic::Response *response);
    static void shareData(int fd, struct Orientation::Response *response);
    static void shareData(int fd, struct Speed::Response *response);

    void requestUltraSonic(uint8_t devices);
    void requestOrientation();
    void requestSpeed();
    void processIncomingMessage(int fd, uint8_t action);
    void processIncomingMessage(uint8_t action);
    bool processUltraSonic(struct UltraSonic::Response *response);
    void processOrientation(struct Orientation::Response *response);
    void processSpeed(struct Speed::Response *response);
    void processRotationComplete();
    void moveStraight(uint8_t direction, uint8_t speed);
    void moveSkew(uint8_t direction, uint8_t speed_left, uint8_t speed_right);
    void rotate(uint8_t direction, float degree);

    States state;
    std::unique_ptr<ConnectionManager> connection_manager;
    int control_pipe[2];
    int base_fd;
    int mapping_fd;
    bool us_pending, orientation_pending, speed_pending, hunt_over;
};

#endif //OBJECT_HUNT_NAVIGATION_NAVIGATOR_H
