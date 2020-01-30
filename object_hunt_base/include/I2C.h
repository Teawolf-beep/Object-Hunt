//
// Created by robert on 05.12.19.
//

#ifndef OBJECT_HUNT_BASE_I2C_H
#define OBJECT_HUNT_BASE_I2C_H

// Project includes
#include "I2CException.h"

const std::string I2C_FILENAME = "/dev/i2c-1"; /**< The absolute path of a valid I²C file. Is mandatory for the I²C-bus
 * to work on the RaspberryPi.*/

/**
 * Custom class for interaction with the I²C-bus.
 * provides initialization as well as read and write access to the I²C bus. This class is only confirmed to work with
 * one I²C slave.
 */
class I2C
{
public:
    /**
     * Custom constructor.
     * Tries to open the passed I²C-file and initializes the slave on the passed address.
     *
     * @param path An absolute path to a valid I²C file.
     * @param slave_address the address of the connected I²C slave.
     * @throw I2CException In case of a failure while initialization.
     */
    I2C(const std::string &path, uint8_t slave_address);

    /**
     * Reads a single byte from the connected I²C device.
     * Reads a single byte at a passed address and returns the content.
     *
     * @param addr The address of the register that will be read.
     * @return The content of the register, -1 in case of an error.
     */
    int16_t readByte(uint8_t addr);

    /**
     * Reads multiple bytes from the connected I²C device.
     * Implements the I²C burst read functionality. A variable length of bytes will be read. The address will be
     * incremented automatically after each reading. Not all I²C devices support this functionality.
     *
     * @param start_addr The starting address of the first register to be read.
     * @param buffer A buffer where the register content will be saved.
     * @param length The number of bytes that should be read.
     * @return The number of bytes successfully read.
     */
    ssize_t readMultiByte(uint8_t start_addr, uint8_t *buffer, size_t length);

    /**
     * Writes bytes to the connected I²C device.
     * Writes a variable count of bytes from the passed buffer to the connected I²C device.
     *
     * @param buffer A buffer containing the bytes t write.
     * @param length The number of bytes that should be written.
     * @return The number of bytes successfully written, -1 in case of an error.
     */
    ssize_t writeBytes(uint8_t *buffer, size_t length);

private:
    ssize_t readOnly(uint8_t *buffer, size_t length);

    int fd;
};

#endif //OBJECT_HUNT_BASE_I2C_H
