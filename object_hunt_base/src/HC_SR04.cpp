//
// Created by robert on 11/22/19.
//

// System includes
#include <cstdint>
#include <chrono>
#include <unistd.h>
#include <cstring>

// Project includes
#include "HC_SR04.h"
#include "HC_SR04Exception.h"
#include "GPIO.h"

// Constructor. Initializes members via member initializer list.
HC_SR04::HC_SR04(std::shared_ptr<GPIO> gpio, uint8_t device, uint8_t trigger, uint8_t echo)
        : gpio(gpio),
        device(device),
        trigger(trigger),
        echo(echo)
{
    // Set the trigger pin as an output
    this->gpio->makeOutput(this->trigger);
    // Initialize a kernel interrupt for the echo pin
    this->echo_pfd.fd = this->gpio->setSingleKernelDriver(this->echo, PullState::OFF, Edge::BOTH);
    // True, if an error occurred while setting up the kernel driver
    if (this->echo_pfd.fd < 0) throw HC_SR04Exception
                ("HC_SR04() -> Error while setting up the echo pin kernel driver for pin " +
                 std::to_string(this->echo) + "!");
    // Add priority events to the pollfd
    this->echo_pfd.events = POLLPRI;
}

// Constructor. Initializes members via member initializer list.
HC_SR04::HC_SR04(std::shared_ptr<GPIO> gpio, uint8_t device, uint8_t trigger,
        uint8_t echo, struct pollfd &&echo_pfd)
        : gpio(gpio),
        device(device),
        trigger(trigger),
        echo(echo),
        echo_pfd(echo_pfd)
{
    // Set the trigger pin as an output
    this->gpio->makeOutput(this->trigger);
}

HC_SR04::~HC_SR04()
{
    // TODO Possible error cause! The measurer thread should be terminated here! Program will crash,
    // if the thread is running!  No time to check it!
    // Join the measurer thread, if possible
    if (this->measurer.joinable()) this->measurer.join();
    // Release the interrupt kernel driver
    this->gpio->releaseSingleKernelDriver(this->echo);
    // Release the associated output
    this->gpio->releaseOutput(this->trigger);
}

void HC_SR04::measure(int fd)
{
    // Join the measurer thread, if possible
    if (this->measurer.joinable()) this->measurer.join();
    // Trigger a new measurement
    this->measurer = std::thread(&HC_SR04::startMeasurement, this, fd);
}

void HC_SR04::startMeasurement(int fd)
{
    char buffer[8];
    std::chrono::steady_clock::time_point start, finish = std::chrono::steady_clock::now();
    // Initial the response struct with values (-2 means a problem occurred). Will be overwritten later, if everything works
    struct InternalUSResponse response = {this->device, 0, 0, 0, -2.};

    // Trigger measurement
    gpio->setPin(this->trigger);
    // The sensor needs the signal for 10 us
    usleep(10);
    gpio->clearPin(this->trigger);

    // Check if the measurement has started. True, if the measurement has not started
    if (poll(&this->echo_pfd, 1, MAX_WAITING_TIME_INITIAL_MS) == -1)
    {
        fprintf(stderr, "Ultra sonic device %d: Failure while polling echo pin", this->device);
        perror("");
        HC_SR04::writeResponse(fd, &response);
        return;
    }
    // True, if the measurement has started
    if (this->echo_pfd.revents & POLLPRI)
    {
        // Get the current time
        start = std::chrono::steady_clock::now();
        // Consume interrupt
        lseek(this->echo_pfd.fd, 0, SEEK_SET);
        // We are not interested in the value of read
        read(this->echo_pfd.fd, buffer, sizeof(buffer));
    }
    // True, if the measurement has not started as expected
    else
    {
        printf("Ultra sonic device %d: Measurement not started as expected.\n", this->device);
        HC_SR04::writeResponse(fd, &response);
        return;
    }
    // Get the elapsed time
    int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::duration<float>(std::chrono::steady_clock::now() - start)).count();
    // Wait for the response of the measurement
    do
    {
        // Wait for an interrupt with time limit. True, if an error occurred
        if (poll(&this->echo_pfd, 1,
                ((MAX_WAITING_TIME_RESPONSE_MS - elapsed) > 0) ? MAX_WAITING_TIME_RESPONSE_MS - elapsed : 0) == -1)
        {
            fprintf(stderr, "Ultra sonic device %d: Failure while polling echo pin", this->device);
            perror("");
            HC_SR04::writeResponse(fd, &response);
            return;
        }
        // True, if an interrupt occurred. This does not mean, that the measurement is over. Poor interrupt detection of RPi.
        if (this->echo_pfd.revents & POLLPRI)
        {
            // Get the current time
            finish = std::chrono::steady_clock::now();
            // Consume the interrupt
            lseek(this->echo_pfd.fd, 0, SEEK_SET);
            read(this->echo_pfd.fd, &buffer, sizeof(buffer));
            // Sleep a short time and check if the pin is really low (that means the measurement is over most likely)
            usleep(5);
            if (!this->gpio->readPin(this->echo)) break;
        }
        // If the code arrives here, a false interrupt was detected. Calculate the maximal possible remaining
        // measurement time
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::duration<float>(std::chrono::steady_clock::now() - start)).count();
    // We need the while loop because of the error prone interrupt detection
    } while(elapsed < MAX_WAITING_TIME_RESPONSE_MS);
    // Calculate time difference between start and end
    std::chrono::duration<float , std::milli> duration = finish - start;
    // No obstacle detected
    if (!this->gpio->readPin(this->echo) && (duration.count() > 0.)) response.distance = (float) (duration.count() * 34/2);
    // Compute distance in cm
    else response.distance = MAX_ULTRA_SONIC_DISTANCE;
    // Write measured distance to the given file descriptor
    HC_SR04::writeResponse(fd, &response);
}

void HC_SR04::writeResponse(int fd, struct InternalUSResponse* response)
{
    // Initialize a pollfd with the response file descriptor and an output event
    struct pollfd pfd = {fd, POLLOUT};
    // Check, if the file descriptor can be written
    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr,"HC_SR04::writeResponse: "
                       "Error while waiting on socket with file descriptor %d: %s", fd, strerror(errno));
        return;
    }
    // Write to the file descriptor
    ssize_t n = write(fd, response, sizeof(InternalUSResponse));
    if (n == -1)
    {
        fprintf(stderr,"HC_SR04::writeResponse: "
                       "Error while writing to socket with file descriptor %d: %s", fd, strerror(errno));
        return;
    }
}
