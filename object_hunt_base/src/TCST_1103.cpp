//
// Created by robert on 11/22/19.
//

// System includes
#include <sys/epoll.h>
#include <cstring>
#include <unistd.h>

// Project includes
#include "TCST_1103.h"
#include "GPIO.h"

TCST_1103::TCST_1103(std::shared_ptr<GPIO> gpio, uint8_t pin)
        : gpio(gpio),
        pin(pin),
        counter(0)
{
    // Initialize interrupt for passed pin
    this->input_fd= this->gpio->setSingleKernelDriver(this->pin, PullState::OFF, Edge::RISING);

    if (pipe(this->listener_pipe_fd))
        throw TCST_1103Exception("TCST_1103() -> Error while creating the receiver pipe " +
                                 std::string(std::strerror(errno)));

    // Start receiver thread
    this->listener = std::thread(&TCST_1103::listen, this);
}

TCST_1103::TCST_1103(std::shared_ptr<GPIO> gpio, uint8_t pin, int fd)
        : gpio(gpio),
        pin(pin),
        input_fd(fd),
        counter(0)
{
    if (pipe(this->listener_pipe_fd))
        throw TCST_1103Exception("TCST_1103() -> Error while creating the receiver pipe " +
                                 std::string(std::strerror(errno)));

    // Start receiver thread
    this->listener = std::thread(&TCST_1103::listen, this);
}

TCST_1103::~TCST_1103()
{
    //Set exit condition and send it to the receiver thread
    bool exit_condition = true;
    write(this->listener_pipe_fd[1], &exit_condition, sizeof (bool));
    // Wait for receiver to terminate
    this->listener.join();

//    printf("~TCST_1103() GPIO count: %ld\n", this->gpio.use_count());

    // Release the interrupt kernel driver
    this->gpio->releaseSingleKernelDriver(this->pin);
}

void TCST_1103::listen()
{
    bool exit_condition = false;
    struct pollfd pfd[2];
    char buffer[8];

    pfd[0].fd = this->listener_pipe_fd[0];
    pfd[0].events = POLLIN;

    pfd[1].fd = this->input_fd;
    pfd[1].events = POLLPRI;

    // Poll file descriptor in endless loop
    while (!exit_condition)
    {
        // Wait endlessly on epoll file descriptor
        if (poll(pfd, 2, -1) == -1)
        {
            fprintf(stderr, "TCST_1103::listen(), pin %d -> "
                            "Error while poll(): %s\n", this->pin, strerror(errno));
            return;
        }
        // True, if a pipe message arrived
        if (pfd[0].revents & POLLIN)
        {
            // Read the input
            ssize_t n = read(listener_pipe_fd[0], &exit_condition, sizeof (exit_condition));
            // True, if an error occurred while reading the input
            if (n < 0)
            {
                fprintf(stderr, "TCST_1103::listen(), pin %d -> "
                                "Error while read(): %s\n", this->pin, strerror(errno));
                return;
            }
        }
        // True, if an interrupt arrived (we are not interested in the content of the message)
        if (pfd[1].revents & POLLPRI)
        {
            ++this->counter;
            lseek(this->input_fd, 0, SEEK_SET);
            read(this->input_fd, buffer, sizeof buffer);
        }
    }
}

uint32_t TCST_1103::getCounter()
{
    return this->counter;
}

void TCST_1103::resetCounter()
{
    this->counter = 0;
}
