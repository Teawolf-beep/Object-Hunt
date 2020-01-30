//
// Created by robert on 04.12.19.
//

#ifndef OBJECT_HUNT_BASE_HARDWAREINTERFACE_H
#define OBJECT_HUNT_BASE_HARDWAREINTERFACE_H

// System includes
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

// Project includes
#include "HardwareInterfaceException.h"

// Forward declarations
class GPIO;
class HC_SR04;
class TCST_1103;
class GearMotor;
class VMA_401;
class BNO_055;
namespace UltraSonic {struct Request;}
namespace Revolution {struct Request;}
namespace Motor {struct Command;}
namespace Stepper {struct Command;}
namespace Speed {struct Request;}
namespace Rotation {struct Command;}

/**
 * Provides methods to control the hardware.
 * Allows hardware interaction for the Object Hunt project.
 */
class HardwareInterface
{
public:
    /**
     * Custom constructor.
     * Will create and initialize objects for all attached hardware parts.
     *
     * @param gpio A shared pointer to a valid GPIO object.
     * @throws HardwareInterfaceException in case an error occurred.
     */
    HardwareInterface(std::shared_ptr<GPIO> gpio);

    /**
     * Custom Destructor.
     * Cleans up. Releases resources and stops all motors.
     */
    ~HardwareInterface();

    /**
     * Enables the motors.
     * The motors needs to be enabled in order to work
     */
    void enableMotors();

    /**
     * Disables the motors.
     * If the motors are disabled, they will not move.
     */
    void disableMotors();

    /**
     * Checks, if the motors are enabled.
     *
     * @return True, if the motors are enabled, false otherwise.
     */
    bool areMotorsEnabled();

    /**
     * Checks, if an ultra sonic measurement is running.
     *
     * @return True, if an ultra sonic measurement is running, false otherwise.
     */
    bool ultraSonicRunning();

    /**
     * Checks, if a rotation movement is running.
     *
     * @return True, if a rotation movement is being performed, false otherwise.
     */
    bool rotationMovementRunning();

    /**
     * Gets the writing end of the rotation control pipe.
     *
     * @return The file descriptor of the writing end.
     */
    int getRotationPipe();

    /**
     * Performs an ultra sonic measurement.
     * Starts a dedicated thread to conduct the measurement. The calling function will return almost immediately,
     * i.e. before the measurement has terminated. The options for the measurement will be dictated by the passed request.
     *
     * @param response_fd A valid file descriptor, to which the results of the measurement will be written.
     * @param request A pointer of type UltraSonic::Request which contains the measurement options.
     */
    void startUltraSonic(int response_fd, struct UltraSonic::Request* request);

    /**
     * Performs a revolution measurement.
     * Gets the revolution count of the devices specified in the request. Writes the result to the passed file descriptor.
     *
     * @param response_fd A valid file descriptor, to which the results of the measurement will be written.
     * @param request A pointer of type Revolution::Request which contains the measurement options.
     */
    void getRevolution(int response_fd, struct Revolution::Request *request);

    /**
     * Performs a speed measurement.
     * Gets the last calculated speed value. Writes the result to the passed file descriptor.
     *
     * @param response_fd A valid file descriptor, to which the results of the measurement will be written.
     * @param id The identifier of the regarding Speed::Request.
     * @return True, if the measurement succeeded, false otherwise.
     */
    bool getSpeed(int response_fd, uint32_t id);

    /**
     * Performs an orientation measurement.
     * Gets the current orientation in degrees (0° - 360°). Writes the result to the passed file descriptor.
     *
     * @param response_fd A valid file descriptor, to which the results of the measurement will be written.
     * @param id The identifier of the regarding Orientation::Request.
     * @return True, if the measurement succeeded, false otherwise.
     */
    bool getOrientation(int response_fd, uint32_t id);

    /**
     * Performs a motor movement.
     * Starts a motor movement with the options specified in the passed command. The motor movement will start
     * immediately without any time limit.
     *
     * @param command A pointer of type Motor::Command which contains the movement options.
     * @return True if the movement started, false otherwise.
     */
    bool moveMotor(struct Motor::Command* command);

    /**
     * Performs a stepper motor movement.
     * Starts a stepper motor movement with the options specified in the passed command.
     * The stepper will move the specified steps and stop afterwards.
     *
     * @param command A pointer of type Stepper::Command which contains the movement options.
     * @return True if the movement started, false otherwise.
     */
    bool moveStepper(struct Stepper::Command* command);

    /**
     * Performs a rotation movement.
     * Starts a rotation movement of the car.
     *
     * @param response_fd A valid file descriptor, to which the resulting orientation of the car will be sent.
     * @param request A pointer of type Rotation::Command which contains the movement options.
     */
    void startRotation(int response_fd, struct Rotation::Command* request);

    /**
     * Stops all attached motors.
     * All attached motors (except the stepper motor) will be stopped immediately.
     */
    void stopMotors();

private:
    void measureUltraSonic(int response_fd, struct UltraSonic::Request* request);
    float getRevolutionMedian();
    void updateSpeed();
    void terminateSpeedAgent();
    void rotateLeft();
    void rotateRight();

    std::shared_ptr<GPIO> gpio;

    int ultra_sonic_pipe[2];
    int speed_pipe[2];
    int rotation_pipe[2];

    std::unique_ptr<BNO_055> bno;

    std::unique_ptr<HC_SR04> us_front;
    std::unique_ptr<HC_SR04> us_rear;
    std::unique_ptr<HC_SR04> us_left;
    std::unique_ptr<HC_SR04> us_right;
    std::atomic<bool> us_running;
    std::mutex write_mutex;
    std::thread us_agent;

    std::unique_ptr<TCST_1103> rev_fr;
    std::unique_ptr<TCST_1103> rev_fl;
    std::unique_ptr<TCST_1103> rev_rr;
    std::unique_ptr<TCST_1103> rev_rl;
    std::atomic<float> speed;
    std::thread speed_agent;

    std::unique_ptr<GearMotor> motor_fr;
    std::unique_ptr<GearMotor> motor_fl;
    std::unique_ptr<GearMotor> motor_rr;
    std::unique_ptr<GearMotor> motor_rl;

    std::unique_ptr<VMA_401> stepper;

    std::atomic<bool> rotation_running;
};

#endif //OBJECT_HUNT_BASE_HARDWAREINTERFACE_H
