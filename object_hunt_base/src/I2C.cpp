//
// Created by robert on 05.12.19.
//

// System includes
#include <fcntl.h>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <string>

// Project includes
#include "I2C.h"

// Constructor. Initializes members via member initializer list.
I2C::I2C(const std::string &path, uint8_t slave_address)
{
    // Opens the I²C file
    this->fd = open(path.c_str(), O_RDWR);
    // True, if an error occurred
    if (this->fd < 0) throw I2CException("I2C() -> Error while opening I2C bus: " + std::string(std::strerror(errno)));
    // Try to set the I²C slave address
    if (ioctl(this->fd, I2C_SLAVE, slave_address) < 0)
        throw I2CException("I2C() -> Error while setting up the I2C slave: " + std::string(std::strerror(errno)));
}

int16_t I2C::readByte(uint8_t addr)
{
    // Buffer for the value
    uint8_t ret;

    // Write a read command to the desired address
    this->writeBytes(&addr, 1);
    // Get the register value. True, if an error occurred
    if (this->readOnly(&ret, 1) != 1) return -1;
    // Return the register value
    return ret;
}

ssize_t I2C::readMultiByte(uint8_t start_addr, uint8_t *buffer, size_t length)
{
    // Write a read command
    this->writeBytes(&start_addr, 1);
    // Read as many bytes as requested
    return this->readOnly(buffer, length);
}

ssize_t I2C::writeBytes(uint8_t *buffer, size_t length)
{
    // Write the content from the passed buffer
    ssize_t ret = write(this->fd, buffer, length);
    // Check for errors
    if (ret < 0) fprintf(stderr,"I2C::writeBytes() -> Error while while write(): %s\n", strerror(errno));
    else if (ret != length) printf("I2C::writeBytes() -> Incomplete message sent!\n");
    // Return the number of written bytes
    return ret;
}

ssize_t I2C::readOnly(uint8_t *buffer, size_t length)
{
    // Read as many bytes as desired
    ssize_t ret = read(this->fd, buffer, length);
    // Check for errors
    if (ret < 0) fprintf(stderr,"I2C::readOnly() -> Error while while read(): %s\n", strerror(errno));
    else if (ret != length) printf("I2C::readOnly() -> Incomplete message received!\n");
    // Return the number of read bytes
    return ret;

}
