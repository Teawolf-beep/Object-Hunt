//
// Created by robert on 05.12.19.
//

// Project includes
#include "BNO_055.h"
#include "I2C.h"

BNO_055::BNO_055(uint8_t address)
        : i2c(std::make_unique<I2C>(I2C_FILENAME, address))
{
    if (this->i2c->readByte(BNO055::Register::CHIP_ID) != BNO055::ID)
        throw BNO_055Exception("BNO055() -> No BNO055 detected on I2C address: " + std::to_string(BNO055::ADDRESS_A));
}

BNO_055::~BNO_055()
{

}

void BNO_055::setOperationMode(uint8_t mode)
{
    uint8_t data[2] = {BNO055::Register::OPR_MODE, mode};
    if (this->i2c->writeBytes(data, 2) != 2)
        printf("BNO055::setOperationMode() -> Failure while setting the operation mode!\n");
}

bool BNO_055::getRawAcceleration(int16_t *buffer)
{
    uint8_t data[6];

    if (this->i2c->readMultiByte(BNO055::Register::ACCEL_DATA_X_LSB, data, 6) != 6)
    {
        printf("BNO055::getRawAcceleration() -> Failure while performing burst read!\n");
        return false;
    }
    buffer[0] = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    buffer[1] = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    buffer[2] = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);
    return true;
}

bool BNO_055::getRawMagnetometer(int16_t *buffer)
{
    uint8_t data[6];

    if (this->i2c->readMultiByte(BNO055::Register::MAG_DATA_X_LSB, data, 6) != 6)
    {
        printf("BNO055::getRawMagnetometer() -> Failure while performing burst read!\n");
        return false;
    }
    buffer[0] = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    buffer[1] = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    buffer[2] = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);
    return true;
}

bool BNO_055::getRawGyroscope(int16_t *buffer)
{
    uint8_t data[6];

    if (this->i2c->readMultiByte(BNO055::Register::GYRO_DATA_X_LSB, data, 6) != 6)
    {
        printf("BNO055::getRawGyroscope() -> Failure while performing burst read!\n");
        return false;
    }
    buffer[0] = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    buffer[1] = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    buffer[2] = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);
    return true;
}

bool BNO_055::getEuler(float *buffer)
{
    uint8_t data[6];

    if (this->i2c->readMultiByte(BNO055::Register::EUL_DATA_X_LSB, data, 6) != 6)
    {
        printf("BNO055::getEuler() -> Failure while performing burst read!\n");
        return false;
    }

    int16_t x = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    int16_t y = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    int16_t z = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);

    buffer[0] = float(x / 16.);
    buffer[1] = float(y / 16.);
    buffer[2] = float(z / 16.);
    return true;
}
