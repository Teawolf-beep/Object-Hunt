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
    if (this->echo_pfd.fd < 0) throw HC_SR04Exception
                ("HC_SR04() -> Error while setting up the echo pin kernel driver for pin " +
                 std::to_string(this->echo) + "!");
    this->echo_pfd.events = POLLPRI;
}

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
    if (this->measurer.joinable()) this->measurer.join();
    // Release the interrupt kernel driver
    this->gpio->releaseSingleKernelDriver(this->echo);
    this->gpio->releaseOutput(this->trigger);
//    printf("~HC_SR04() GPIO count: %ld\n", this->gpio.use_count());
}

void HC_SR04::measure(int fd)
{
    if (this->measurer.joinable()) this->measurer.join();
    this->measurer = std::thread(&HC_SR04::startMeasurement, this, fd);
}

void HC_SR04::startMeasurement(int fd)
{
    char buffer[8];
    std::chrono::steady_clock::time_point start, finish = std::chrono::steady_clock::now();
    struct InternalUSResponse response = {this->device, 0, 0, 0, -2.};

    // Trigger measurement
    gpio->setPin(this->trigger);
    usleep(10);
    gpio->clearPin(this->trigger);

    // Check if the measurement has started
    if (poll(&this->echo_pfd, 1, MAX_WAITING_TIME_INITIAL_MS) == -1)
    {
        fprintf(stderr, "Ultra sonic device %d: Failure while polling echo pin", this->device);
        perror("");
        HC_SR04::writeResponse(fd, &response);
        return;
    }
    if (this->echo_pfd.revents & POLLPRI)
    {
        start = std::chrono::steady_clock::now();
        lseek(this->echo_pfd.fd, 0, SEEK_SET);
        read(this->echo_pfd.fd, buffer, sizeof(buffer));
    }
    else
    {
        printf("Ultra sonic device %d: Measurement not started as expected.\n", this->device);
        HC_SR04::writeResponse(fd, &response);
        return;
    }
    int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::duration<float>(std::chrono::steady_clock::now() - start)).count();
    // Wait for the response of the measurement
    do
    {
        if (poll(&this->echo_pfd, 1,
                ((MAX_WAITING_TIME_RESPONSE_MS - elapsed) > 0) ? MAX_WAITING_TIME_RESPONSE_MS - elapsed : 0) == -1)
        {
            fprintf(stderr, "Ultra sonic device %d: Failure while polling echo pin", this->device);
            perror("");
            HC_SR04::writeResponse(fd, &response);
            return;
        }
        if (this->echo_pfd.revents & POLLPRI)
        {
            finish = std::chrono::steady_clock::now();
            lseek(this->echo_pfd.fd, 0, SEEK_SET);
            read(this->echo_pfd.fd, &buffer, sizeof(buffer));
            usleep(5);
            if (!this->gpio->readPin(this->echo)) break;
        }
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::duration<float>(std::chrono::steady_clock::now() - start)).count();
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
    struct pollfd pfd = {fd, POLLOUT};

    if (poll(&pfd, 1, -1) == -1)
    {
        fprintf(stderr,"HC_SR04::writeResponse: "
                       "Error while waiting on socket with file descriptor %d: %s", fd, strerror(errno));
        return;
    }
    ssize_t n = write(fd, response, sizeof(InternalUSResponse));
    if (n == -1)
    {
        fprintf(stderr,"HC_SR04::writeResponse: "
                       "Error while writing to socket with file descriptor %d: %s", fd, strerror(errno));
        return;
    }
}
