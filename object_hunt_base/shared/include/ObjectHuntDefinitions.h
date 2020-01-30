//
// Created by robert on 04.12.19.
//

#ifndef OBJECT_HUNT_BASE_OBJECTHUNTDEFINITIONS_H
#define OBJECT_HUNT_BASE_OBJECTHUNTDEFINITIONS_H

#include <cstdint>

/**
 * Contains definitions for sockets.
 * Provides definitions with regard to sockets.
 */
namespace Socket
{
    /**
     * Contains definitions for ports.
     * Provides definitions with regard to ports.
     */
    namespace Ports
    {
        const uint16_t BASE = 19001; /**< Base process port. The base process is listening on this port for a connection from the navigation process. */
        const uint16_t NAVIGATION = 19002; /**< Navigation process port. The navigation process provides this socket as a connection point for all other processes. */
    }

    /**
     * Contains definitions for socket settings.
     * Provides definitions with regard to socket settings.
     */
    namespace Settings
    {
        const int QUEUE_LENGTH = 5; /**< Maximal queue length. The maximal number of processes that can be queued on a socket. */
        const uint8_t MAX_CONNECTIONS = 32; /**< Maximal connections. The maximal number of connections the navigation process can handle. */
    }
}

/**
 * The return value of the base process.
 * The main method decides depending on these return values of the application class, how or if the process will be terminated.
 */
namespace ReturnValue
{
    const int8_t COMMON_ERROR = -3; /**< Generic error. An error occurred while executing the program. */
    const int8_t NO_CONNECTION = -2; /**< No connection established. The socket connection to the navigation process could not be established. */
    const int8_t POWER_OFF = -1; /**< Shutdown the RaspberryPi. The start/ stop button was pressed. This return value will cause the RaspberryPi to shutdown. */
    const int8_t LEAVE_SUCCESS = 0; /**< No special events. The program terminated ordinary. */
}

/**
 * Contains definitions for buttons.
 * Provides definitions for the start/ stop button.
 */
namespace Button
{
    const uint8_t START_STOP = 21; /**< BCM number. The GPIO pin number for the start/ stop button */
}

/**
 * Contains definitions for action identifiers.
 * Action identifiers are sent before every command, request or response. They help the receiving program to estimate
 * and classify the following data.
 */
namespace Action
{
    const uint8_t MOTOR = 0; /**< Motor action. The following data will be a motor command.*/
    const uint8_t STEPPER = 1; /**< Stepper action. The following data will be a stepper command.*/
    const uint8_t ULTRA_SONIC = 2; /**< Ultra sonic action. The following data will be an ultra sonic request or response.*/
    const uint8_t REVOLUTION = 3; /**< Revolution action. The following data will be a revolution request or response.*/
    const uint8_t SPEED = 4; /**< Speed action. The following data will be a value request or response.*/
    const uint8_t ROTATION = 5; /**< Rotation action. The following data will be a rotation request or response*/
    const uint8_t ORIENTATION = 6; /**< Orientation action. The following data will be an orientation request or response*/
    const uint8_t START = 7; /**< Start action. Tells the receiving process to start the object hunt. No data will follow.*/
    const uint8_t TERMINATE = 8; /**< Terminate action. Tells the receiving process to terminate the object hunt. No data will follow.*/
    const uint8_t IDENTIFIER_MAPPING = 9; /**< Mapping identifier action. Identifies the sending process as the mapping process. No data will follow.*/
    const uint8_t IDENTIFIER_CAMERA = 10; /**< Camera identifier action. Identifies the sending process as the camera process. No data will follow.*/
    const uint8_t OBJECT_DETECTED = 11; /**< Object detected action. Tells the receiving process that the object was found and the hunt is over. No data will follow.*/
    const uint8_t ROTATION_COMPLETE = 12; /**< Rotation complete action. Tells the receiving process that the rotation command has succeeded.*/
    const uint8_t OBJECT_DETECTED_ACK = 13; /**< Acknowledges the object detection. Tells the camera process that the object detected message was received successfully.*/
}

/**
 * Contains definitions for motors.
 * Defines commands for the inter-process communication, pin numbers to interfere with the hardware and more.
 * Left and right should be seen from the perspective of an imaginary driver of the car.
 */
namespace Motor
{
    /**
     * Used to control the motors.
     * The base process will control the motors based on commands of this type from the navigation process.
     */
    struct Command
    {
        uint8_t devices; /**< The devices for which the command applies. Each of the four LSB represent one regarding motor.*/
        uint8_t direction; /**< Specifies the direction. Please see the Direction namespace for possible values.*/
        uint8_t duty_cycle; /**< Specifies the duty cycle. The duty cycle translates into the value of the
                                * regarding motor. Possible values are 0-100. */
    };

    /**
     * The mapping of each bit to the respecting motor.
     * Each motor device is represented through a regarding bit.
     */
    namespace Device
    {
        const uint8_t FRONT_RIGHT = 0x01; /**< The front right motor.*/
        const uint8_t FRONT_LEFT = 0x02; /**< The front left motor.*/
        const uint8_t BACK_RIGHT = 0x04; /**< The back right motor.*/
        const uint8_t BACK_LEFT = 0x08; /**< The back left motor.*/
    }

    /**
     * The direction definitions.
     * Each direction (forward, backward) and not moving is represented through a regarding bit.
     */
    namespace Direction
    {
        const uint8_t FORWARD = 0x01; /**< Forward. Should not need further explanation.*/
        const uint8_t BACKWARD = 0x02; /**< Backward. Should not need further explanation.*/
        const uint8_t NOT_MOVING = 0x04; /**< No movement. Mainly used in requests of the current rotation direction of the regarding motor.*/
    }

    /**
     * The pin definitions.
     * Each motor is controlled by two GPIOs. A high level lets the regarding motor turn forward or backward.
     * If one of the two pins is high, the other pin should be set low. Otherwise the drivers will behave undefined.
     */
    namespace Pin
    {
        // M1
        const uint8_t FR_FORWARD =  7; /**< Front right forward.*/
        const uint8_t FR_BACKWARD = 11; /**< Front right backward.*/
        // M2
        const uint8_t FL_FORWARD =  1; /**< Front left forward.*/
        const uint8_t FL_BACKWARD = 0; /**< Front right backward.*/
        // M3
        const uint8_t RL_FORWARD =  6; /**< Rear left forward.*/
        const uint8_t RL_BACKWARD = 13; /**< Rear right backward.*/
        // M4
        const uint8_t RR_FORWARD =  19; /**< Rear right forward.*/
        const uint8_t RR_BACKWARD = 16; /**< Rear right backward.*/
    }
    const uint8_t ENABLE_ALL = 20; /**< Enable all motors. This pin has to be set high in order for the motors for move.
                                        * If this pin is low, the motor drivers will not move any motor.*/
    const uint32_t PWM_FREQUENCY = 1000; /**< The motors PWM frequency. The value represents the PWM frequency value in hertz.*/
}

/**
 * Contains definitions for the stepper motor.
 * Defines commands for the inter-process communication, pin numbers to interfere with the hardware and more.
 * Left and right should be seen from the perspective of an imaginary driver of the car. The stepper motor is used to mount and move
 * the camera.
 */
namespace Stepper
{
    /**
     * Used to control the stepper motor.
     * The base process will control the stepper motor based on commands of this type from the navigation process.
     */
    struct Command
    {
        uint8_t direction; /**< The direction of the stepper motor. Please see the Direction namespace for possible values.*/
        uint8_t steps; /**< The steps to go. This field contains the steps the stepper motor shall move.*/
    };

    /**
     * The direction definitions.
     * Each direction (left, right) is represented by an unsigned number.
     */
    namespace Direction
    {
        const uint8_t LEFT = 0; /**< Left direction. The direction should be seen from the perspective of an imaginary driver of the car.*/
        const uint8_t RIGHT = 1; /**< Right direction. The direction should be seen from the perspective of an imaginary driver of the car.*/
    }

    /**
     * The pin definitions.
     * Each end of the two motor coils can be controlled directly. Please see https://www.velleman.eu/downloads/29/vma401_a4v01.pdf
     * (page 4) for further information on how to understand the numbering.
     */
    namespace Pin
    {
        // Blue
        const uint8_t ONE = 8; /**< Blue wire. Please see https://www.velleman.eu/downloads/29/vma401_a4v01.pdf (page 4) for further information.*/
        // Pink
        const uint8_t TWO = 9; /**< Pink wire. Please see https://www.velleman.eu/downloads/29/vma401_a4v01.pdf (page 4) for further information.*/
        // Yellow
        const uint8_t THREE = 25; /**< Yellow wire. Please see https://www.velleman.eu/downloads/29/vma401_a4v01.pdf (page 4) for further information.*/
        // Orange
        const uint8_t FOUR = 10; /**< Orange wire. Please see https://www.velleman.eu/downloads/29/vma401_a4v01.pdf (page 4) for further information.*/
    }
    const uint16_t COUNTS_PER_REV = 512; /**< Counts pre revolution. After 512 steps the motor has done one complete revolution.*/
    const uint8_t MAX_STEPS = COUNTS_PER_REV/4; /**< Maximum steps per command. Is used in order to not accidentally damage the camera wire.*/
    const uint16_t IDLE_TIME = 1000; /**< Idle time between steps. With this value the value of the stepper motor can be controlled.*/
    const uint8_t LOOKUP[8] = {0b01000, 0b01100, 0b00100, 0b00110, 0b00010, 0b00011, 0b00001, 0b01001}; /**< Lookup table.
 * This table is used control the stepper motor coils in the right order.*/
}

/**
 * Contains definitions for the HC SR04 ultra sonic sensors.
 * Defines requests and responses for the inter-process communication, pin numbers to interfere with the hardware and more.
 */
namespace UltraSonic
{

    /**
     * Ultra sonic measurement request.
     * The base process will conduct ultra sonic measurements based on requests of this type.
     */
    struct Request
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Has to be maintained by the requesting process.*/
        uint8_t pad__; /**< Padding byte.*/
        uint8_t res0__; /**< Padding/ reserve byte.*/
        uint8_t res1__; /**< Padding/ reserve byte. */
        uint8_t devices; /**< The devices which will be triggered by the request. The four LSB represent each one ultra sonic sensor.
 * Please see the namespace UltraSonic::Device for possible values.*/
    };

    /**
     * Ultra sonic measurement response.
     * The base process will answer to each valid ultra sonic request with a response of this type.
     */
    struct Response
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        float distance_front; /**< Front sensor distance. Shows the distance to the next obstacle in cm (maximal value is defined in MAX_ULTRA_SONIC_DISTANCE).
 * Contains a negative value, if the sensor was not requested (-1.0) or an error occurred (-2.0) while conducting the measurement. */
        float distance_rear; /**< Rear sensor distance. Shows the distance to the next obstacle in cm (maximal value is defined in MAX_ULTRA_SONIC_DISTANCE).
 * Contains a negative value, if the sensor was not requested (-1.0) or an error occurred (-2.0) while conducting the measurement. */
        float distance_left; /**< Left sensor distance. Shows the distance to the next obstacle in cm (maximal value is defined in MAX_ULTRA_SONIC_DISTANCE).
 * Contains a negative value, if the sensor was not requested (-1.0) or an error occurred (-2.0) while conducting the measurement. */
        float distance_right; /**< Right sensor distance. Shows the distance to the next obstacle in cm (maximal value is defined in MAX_ULTRA_SONIC_DISTANCE).
 * Contains a negative value, if the sensor was not requested (-1.0) or an error occurred (-2.0) while conducting the measurement. */
    };

    /**
     * The mapping of each bit to the respecting ultra sonic sensor.
     * Each ultra sonic sensor is represented through a regarding bit.
     */
    namespace Device
    {
        const uint8_t FRONT = 0x01; /**< The ultra sonic sensor in the front.*/
        const uint8_t REAR = 0x02; /**< The ultra sonic sensor in the rear.*/
        const uint8_t LEFT = 0x04; /**< The ultra sonic sensor on the left.*/
        const uint8_t RIGHT = 0x08; /**< The ultra sonic sensor on the right.*/
    }

    /**
     * The pin definitions.
     * Each ultra sonic sensor has one trigger and one echo pin. Both pins a re needed in order to conduct a measurement.
     */
    namespace Pin
    {
        const uint8_t FRONT_TRIGGER = 4; /**< Trigger pin front. This pin is used in order to start an ultra sonic measurement in the front.*/
        const uint8_t FRONT_ECHO = 23; /**< Echo pin front. The time between two edges on this pin is used to calculate the distance.*/
        const uint8_t REAR_TRIGGER = 18; /**< Trigger pin rear. This pin is used in order to start an ultra sonic measurement in the rear.*/
        const uint8_t REAR_ECHO = 17; /**< Echo pin rear. The time between two edges on this pin is used to calculate the distance.*/
        const uint8_t LEFT_TRIGGER = 5; /**< Trigger pin left. This pin is used in order to start an ultra sonic measurement on the left.*/
        const uint8_t LEFT_ECHO = 22; /**< Echo pin left. The time between two edges on this pin is used to calculate the distance.*/
        const uint8_t RIGHT_TRIGGER = 12; /**< Trigger pin right. This pin is used in order to start an ultra sonic measurement on the right.*/
        const uint8_t RIGHT_ECHO = 27; /**< Echo pin right. The time between two edges on this pin is used to calculate the distance.*/
    }
}

/**
 * Contains definitions for the revolution sensors.
 * Defines requests and responses for the inter-process communication, pin numbers to interfere with the hardware and more.
 */
namespace Revolution
{
    /**
     * Revolution request.
     * The base process will obtain the current counter value based on requests of this type.
     */
    struct Request
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Has to be maintained by the requesting process.*/
        uint8_t pad__; /**< Padding byte.*/
        uint8_t res0__; /**< Padding/ reserve byte.*/
        uint8_t res1__; /**< Padding/ reserve byte.*/
        uint8_t devices; /**< The devices which will be read out by the request. The four LSB represent each one ultra sonic sensor.*/
    };

    /**
     * Revolution response.
     * The base process will answer to each valid revolution request with a response of this type.
     */
    struct Response
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        int32_t count_front_left; /**< Count of the front left revolution sensor. Each full revolution of the regarding wheel is represented by 20 steps.*/
        int32_t count_front_right; /**< Count of the front right revolution sensor. Each full revolution of the regarding wheel is represented by 20 steps.*/
        int32_t count_rear_left; /**< Count of the rear left revolution sensor. Each full revolution of the regarding wheel is represented by 20 steps.*/
        int32_t count_rear_right; /**< Count of the rear right revolution sensor. Each full revolution of the regarding wheel is represented by 20 steps.*/
    };

    /**
     * The mapping of each bit to the respecting revolution sensor.
     * Each revolution sensor is represented through a regarding bit.
     */
    namespace Device
    {
        const uint8_t FRONT_RIGHT = 0x1; /**< The front right revolution sensor.*/
        const uint8_t FRONT_LEFT = 0x2; /**< The front left revolution sensor.*/
        const uint8_t REAR_RIGHT = 0x4; /**< The rear right revolution sensor.*/
        const uint8_t REAR_LEFT = 0x8; /**< The rear left revolution sensor.*/
    }

    /**
     * The pin definitions.
     * Each revolution sensor is mapped to one respecting GPIO. The base process is counting the edges in order to
     * calculate value and travelled distance.
     */
    namespace Pin
    {
        const uint8_t FRONT_RIGHT = 14; /**< Pin front right.*/
        const uint8_t FRONT_LEFT = 15; /**< Pin front left.*/
        const uint8_t REAR_LEFT = 24; /**< Pin rear left.*/
        const uint8_t REAR_RIGHT = 26; /**< Pin rear right.*/
    }
}

/**
 * Contains definitions regarding speed.
 * Defines requests and responses for the inter-process communication and definitions to calculate the speed.
 */
namespace Speed
{
    const int MEASUREMENT_INTERVAL_MS = 250; /**< The measurement interval. A dedicated thread will calculate the actual speed in this interval.*/
    const float WHEEL_DIAMETER_CM = 6.61; /**< Diameter of the wheels. The diameter of the wheels in centimeter.*/
    const float WHEEL_EXTENT_CM = float(3.141592654 * WHEEL_DIAMETER_CM); /**< Extent of the wheels. The extent of the wheels in centimeter.*/
    const uint8_t COUNTS_PER_REVOLUTION = 20; /**< Counts per revolution. The number of counts that indicates a full revolution of the respecting wheel.*/
    const float DISTANCE_PER_COUNT_CM = WHEEL_EXTENT_CM / COUNTS_PER_REVOLUTION; /**< Calculated distance per count. The distance that each count translates to in centimeter.*/

    /**
     * Speed request.
     * The base process will read the last calculated speed value based on a request of this type.
     */
    struct Request
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Has to be maintained by the requesting process.*/
    };

    /**
     * Speed response.
     * The base process will answer to each valid value request with a response of this type.
     */
    struct Response
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        float value; /**< The speed value. Represents the las calculated speed value in centimeters per second.*/
    };

    /**
     * Speed response for the mapping process.
     * Has the same fields as Speed::Response with an additional field for the time difference with regard to the last message of this type.
     */
    struct ResponseMapping
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        float value; /**< The speed value. Represents the las calculated speed value in centimeters per second.*/
        float time_diff_ms; /**< The time difference to the last transmitted speed measurement in milliseconds.*/
    };
}

/**
 * Contains definitions regarding rotation.
 * Defines commands for the inter-process communication and more.
 */
namespace Rotation
{
    const int ROTATION_AGENT_TIMEOUT_MS = 5; /**< Measurement timeout in milliseconds. If a valid rotation request arrived the base process,
 * it will start a rotation movement and check the current rotation angle in this interval.*/
    const uint8_t DUTY_CYCLE = 90; /**< Duty cycle for the motors. Each rotation movement will be performed with this duty cycle.*/

    /**
     * Rotation command.
     * The base process will perform a rotation movement based on a command of this type.
     */
    struct Command
    {
        uint8_t direction; /**< The direction of the rotation movement. Please see the Direction namespace for possible values.*/
        uint8_t pad__; /**< Padding byte.*/
        uint8_t res0__; /**< Padding/ reserve byte.*/
        uint8_t res1__; /**< Padding/ reserve byte.*/
        float degree; /**< The degrees to rotate. The car will rotate the specified amount of degrees.*/
    };

    /**
     * The direction definitions.
     * Each direction (left, right) is represented by an unsigned number.
     */
    namespace Direction
    {
        const uint8_t LEFT = 0; /**< Left direction. The direction should be seen from the perspective of an imaginary driver of the car.*/
        const uint8_t RIGHT = 1; /**< Right direction. The direction should be seen from the perspective of an imaginary driver of the car.*/
    }
}

/**
 * Contains definitions regarding the orientation.
 * Defines requests and responses for the inter-process communication.
 */
namespace Orientation
{
    /**
     * Orientation request.
     * The base process will perform an orientation measurement based on a request of this type.
     */
    struct Request
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Has to be maintained by the requesting process.*/
    };

    /**
     * Orientation response.
     * The base process will answer to each valid value request with a response of this type.
     */
    struct Response
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        float xyz[3]; /**< XYZ data array. The X field (xyz[0]) contains the heading of the car. The Y field (xyz[1]) contains the roll of the car.
 * The Z field (xyz[2]) contains the pitch of hte car.*/
    };

    /**
     * Orientation response for the mapping process.
     * Has the same fields as Orientation::Response with an additional field for the time difference with regard to the last message of this type.
     */
    struct ResponseMapping
    {
        uint32_t id; /**< Identifier. Is used to map a response to the regarding request. Will be the same identifier as in the respecting request.*/
        float xyz[3]; /**< XYZ data array. The X field (xyz[0]) contains the heading of the car. The Y field (xyz[1]) contains the roll of the car.
 * The Z field (xyz[2]) contains the pitch of hte car.*/
        float time_diff_ms; /**< The time difference to the last transmitted orientation measurement in milliseconds.*/
    };
}

/**
 * Contains definitions regarding the inter integrated circuit.
 * Defines pin assignments (BCM) to interfere with the hardware.
 */
namespace InterIntegratedCircuit
{
    const uint8_t SDA = 2; /**< Data pin.*/
    const uint8_t SCL = 3; /**< Clock pin.*/
}

#endif //OBJECT_HUNT_BASE_OBJECTHUNTDEFINITIONS_H
