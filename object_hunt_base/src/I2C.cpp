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

I2C::I2C(const std::string &path, uint8_t slave_address)
{
    this->fd = open(path.c_str(), O_RDWR);
    if (this->fd < 0) throw I2CException("I2C() -> Error while opening I2C bus: " + std::string(std::strerror(errno)));
    if (ioctl(this->fd, I2C_SLAVE, slave_address) < 0)
        throw I2CException("I2C() -> Error while setting up the I2C slave: " + std::string(std::strerror(errno)));
}

int16_t I2C::readByte(uint8_t addr)
{
    uint8_t ret;

    this->writeBytes(&addr, 1);
    if (this->readOnly(&ret, 1) != 1) return -1;
    return ret;
}

ssize_t I2C::readMultiByte(uint8_t start_addr, uint8_t *buffer, size_t length)
{
    this->writeBytes(&start_addr, 1);
    return this->readOnly(buffer, length);
}

ssize_t I2C::writeBytes(uint8_t *buffer, size_t length)
{
    ssize_t ret = write(this->fd, buffer, length);
    if (ret < 0) fprintf(stderr,"I2C::writeBytes() -> Error while while write(): %s\n", strerror(errno));
    else if (ret != length) printf("I2C::writeBytes() -> Incomplete message sent!\n");
    return ret;
}

ssize_t I2C::readOnly(uint8_t *buffer, size_t length)
{
    ssize_t ret = read(this->fd, buffer, length);
    if (ret < 0) fprintf(stderr,"I2C::readOnly() -> Error while while read(): %s\n", strerror(errno));
    else if (ret != length) printf("I2C::readOnly() -> Incomplete message received!\n");
    return ret;

}
