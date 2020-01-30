//
// Created by robert on 11/22/19.
//

#ifndef OBJECT_HUNT_BASE_HC_SR04_H
#define OBJECT_HUNT_BASE_HC_SR04_H

// System includes
#include <memory>
#include <poll.h>
#include <atomic>
#include <future>
#include <thread>

// Forward declarations
class GPIO;
class TCPSocket;

const uint8_t MAX_WAITING_TIME_INITIAL_MS = 3; /**< Maximum ultra sonic initial waiting time. If an ultra sonic measurement has not started
 * after this amount of time, the measurement will be terminated and fail.*/
const uint8_t MAX_WAITING_TIME_RESPONSE_MS = 25; /**< Maximum ultra sonic response waiting time. If an ultra sonic measurement is started and the
 * response signal has not been received after this amount of time, the maximum distance (MAX_ULTRA_SONIC_DISTANCE) will be set as the distance value.*/
const float MAX_ULTRA_SONIC_DISTANCE = 425.; /**< The maximum possible ultra sonic distance. If the ultra sonic measurement response
 * is not recognized in time, this distance will be set. The distance can be bigger in reality.*/

/**
 * Defines the internal ultra sonic response.
 * If an ultra sonic measurement is conducted, the involved threads will communicate with a structure of this type.
 */
struct InternalUSResponse
{
    uint8_t device; /**< The device which is sending the message. Please see the namespace UltraSonic::Device for possible values.*/
    uint8_t pad__; /**< Padding byte.*/
    uint8_t res0__; /**< Padding/ reserve byte.*/
    uint8_t res1__; /**< Padding/ reserve byte. */
    float distance; /**< Distance of the regarding device in cm. The maximal possible value is defined in MAX_ULTRA_SONIC_DISTANCE*/
};

/**
 * A class for the HC-SR04 ultra sonic sensor.
 * Helps with setting up the sensor and taking measurements.
 */
class HC_SR04
{
public:

    /**
     * Custom constructor.
     * Sets up the given pins for trigger and echo on its own. Should be preferred, if only one HC SR04 sensor is used.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param device One of the bits defined in UltraSonic::Device to identify this unit.
     * @param trigger The regarding trigger pin.
     * @param echo The regarding echo pin.
     * @throw HC_SR04Exception in case something went wrong.
     */
    HC_SR04(std::shared_ptr<GPIO> gpio, uint8_t device, uint8_t trigger, uint8_t echo);

    /**
     * Custom constructor.
     * Expects an interrupt for the echo pin to be already configured. If multiple HC SR04 sensors are going to be used,
     * this constructor is more time efficient as all echo pins can be configured together before constructing the objects.
     *
     * @param gpio A shared pointer to a GPIO object.
     * @param device One of the bits defined in UltraSonic::Device to identify this unit.
     * @param trigger The regarding trigger pin.
     * @param echo The regarding echo pin.
     * @param echo_pfd A valid poll file descriptor configured for the echo pin.
     * @throw HC_SR04Exception in case something went wrong.
     */
    HC_SR04(std::shared_ptr<GPIO> gpio, uint8_t device, uint8_t trigger, uint8_t echo, struct pollfd &&echo_pfd);

    /**
     * Custom destructor.
     * Releases the trigger pin and the interrupt driver. Joins the measurer thread before termination.
     */
    ~HC_SR04();

    /**
     * Public method to trigger a measurement.
     * Will start a measurement as soon as possible. The measurement is conducted by a dedicated thread. The calling
     * thread will return from this function, before the measurement is completed. Sends the measured distance in a
     * struct of type InternalUSResponse to the passed file descriptor.
     *
     * @param fd A valid file descriptor to which the result will be passed.
     */
    void measure(int fd);

private:
    static void writeResponse(int fd, struct InternalUSResponse* response);
    void startMeasurement(int fd);

    uint8_t device;
    uint8_t trigger;
    uint8_t echo;
    std::shared_ptr<GPIO> gpio;
    struct pollfd echo_pfd;
    std::thread measurer;
};
#endif //OBJECT_HUNT_BASE_HC_SR04_H
