//
// Created by robert on 05.12.19.
//

#ifndef OBJECT_HUNT_BASE_BNO_055_H
#define OBJECT_HUNT_BASE_BNO_055_H

// System includes
#include <cstdint>
#include <memory>

// Project includes
#include "BNO_055Exception.h"

/**
 * Contains definitions regarding the BNO055.
 * The BNO055 is an intelligent absolute orientation sensor produced by Bosch.
 */
namespace BNO055
{
    const uint8_t ADDRESS_A = 0x28; /**< Device address A. Ths is the default device address. The device address can be altered by setting the ADR pin high.*/
    const uint8_t ADDRESS_B = 0x29; /**< Device address B. Ths is the alternative device address. The device address can be altered by setting the ADR pin high.*/
    const uint8_t ID = 0xA0; /**< Identifier. Identifier of the sensor. Is stored in the CHIP_ID register and can be controlled to verify correct communication.*/
    /**
     * Contains definitions regarding the BNO055 operation mode.
     * The BNO055 can be set in 13 different operation mode. Please use the function setOperationMode().
     */
    namespace OperationMode
    {
        const uint8_t CONFIGMODE = 0x00; /**< Operation mode CONFIGMODE.*/
        const uint8_t ACCONLY = 0x01; /**< Operation mode ACCONLY.*/
        const uint8_t MAGONLY = 0X02; /**< Operation mode MAGONLY.*/
        const uint8_t GYRONLY = 0X03; /**< Operation mode GYRONLY.*/
        const uint8_t ACCMAG = 0X04; /**< Operation mode ACCMAG.*/
        const uint8_t ACCGYRO = 0X05; /**< Operation mode ACCGYRO.*/
        const uint8_t MAGGYRO = 0X06; /**< Operation mode MAGGYRO.*/
        const uint8_t AMG = 0X07; /**< Operation mode AMG.*/
        const uint8_t IMUPLUS = 0X08; /**< Operation mode IMUPLUS. Used in this project.*/
        const uint8_t COMPASS = 0X09; /**< Operation mode COMPASS.*/
        const uint8_t M4G = 0X0A; /**< Operation mode M4G.*/
        const uint8_t NDOF_FMC_OFF = 0X0B; /**< Operation mode NDOF_FMC_OFF.*/
        const uint8_t NDOF = 0X0C; /**< Operation mode NDOF.*/
    }
    /**
     * A complete register map.
     * All adresses with a brief description of th register function. Please see
     * https://www.bosch-sensortec.com/media/boschsensortec/downloads/smart_sensors_1/bno055/bst-bno055-ds000.pdf
     * for further information.
     */
    namespace Register
    {
        const uint8_t CHIP_ID = 0x00; /**< BNO055 chip ID. Chip identification code, read-only fixed value 0xA0.*/
        const uint8_t ACCEL_REV_ID = 0x01; /**< ACC chip ID. Chip ID of the Accelerometer device, read-only fixed value 0xFB.*/
        const uint8_t MAG_REV_ID = 0x02; /**< MAG chip ID. Chip ID of the Magnetometer device, read-only fixed value 0x32.*/
        const uint8_t GYRO_REV_ID = 0x03; /**< GRYO chip ID. ChipID of the Gyroscope device, read-only fixed value 0x0F.*/
        const uint8_t SW_REV_ID_LSB = 0x04; /**< SW Revision ID <7:0>. Lower byte of SW Revision ID, read-only fixed value depending on SW revision programmed on microcontroller.*/
        const uint8_t SW_REV_ID_MSB = 0x05; /**< SW Revision ID <15:8>. Upper byte of SW Revision ID, read-only fixed value depending on SW revision programmed on microcontroller.*/
        const uint8_t BL_REV_ID = 0X06; /**< Bootloader Version. Identifies the version of the bootloader in the microcontroller, read-only.*/

        /* Accel data register */
        const uint8_t ACCEL_DATA_X_LSB = 0X08; /**< Acceleration Data X <7:0>.*/
        const uint8_t ACCEL_DATA_X_MSB = 0X09; /**< Acceleration Data X <15:8>.*/
        const uint8_t ACCEL_DATA_Y_LSB = 0X0A; /**< Acceleration Data Y <7:0>.*/
        const uint8_t ACCEL_DATA_Y_MSB = 0X0B; /**< Acceleration Data Y <15:8>.*/
        const uint8_t ACCEL_DATA_Z_LSB = 0X0C; /**< Acceleration Data Z <7:0>.*/
        const uint8_t ACCEL_DATA_Z_MSB = 0X0D; /**< Acceleration Data Z <15:8>.*/

        /* Mag data register */
        const uint8_t MAG_DATA_X_LSB = 0X0E; /**< Magnetometer Data X <7:0>.*/
        const uint8_t MAG_DATA_X_MSB = 0X0F; /**< Magnetometer Data X <15:8>.*/
        const uint8_t MAG_DATA_Y_LSB = 0X10; /**< Magnetometer Data Y <7:0>.*/
        const uint8_t MAG_DATA_Y_MSB = 0X11; /**< Magnetometer Data Y <15:8>.*/
        const uint8_t MAG_DATA_Z_LSB = 0X12; /**< Magnetometer Data Z <7:0>.*/
        const uint8_t MAG_DATA_Z_MSB = 0X13; /**< Magnetometer Data Z <15:8>.*/

        /* Gyro data registers */
        const uint8_t GYRO_DATA_X_LSB = 0X14; /**< Gyroscope Data X <7:0>.*/
        const uint8_t GYRO_DATA_X_MSB = 0X15; /**< Gyroscope Data X <15:8>.*/
        const uint8_t GYRO_DATA_Y_LSB = 0X16; /**< Gyroscope Data Y <7:0>.*/
        const uint8_t GYRO_DATA_Y_MSB = 0X17; /**< Gyroscope Data Y <15:8>.*/
        const uint8_t GYRO_DATA_Z_LSB = 0X18; /**< Gyroscope Data Z <7:0>.*/
        const uint8_t GYRO_DATA_Z_MSB = 0X19; /**< Gyroscope Data Z <15:8>.*/

        /* Euler data registers */
        const uint8_t EUL_DATA_X_LSB = 0X1A; /**< Heading Data <7:0>.*/
        const uint8_t EUL_DATA_X_MSB = 0X1B; /**< Heading Data <15:8>.*/
        const uint8_t EUL_DATA_Y_LSB = 0X1C; /**< Roll Data <7:0>.*/
        const uint8_t EUL_DATA_Y_MSB = 0X1D; /**< Roll Data <15:8>.*/
        const uint8_t EUL_DATA_Z_LSB = 0X1E; /**< Pitch Data <7:0>.*/
        const uint8_t EUL_DATA_Z_MSB = 0X1F; /**< Pitch Data <15:8>.*/

        /* Quaternion data registers */
        const uint8_t QUATERNION_DATA_W_LSB = 0X20; /**< Quaternion w Data <7:0>.*/
        const uint8_t QUATERNION_DATA_W_MSB = 0X21; /**< Quaternion w Data <15:8>.*/
        const uint8_t QUATERNION_DATA_X_LSB = 0X22; /**< Quaternion x Data <7:0>.*/
        const uint8_t QUATERNION_DATA_X_MSB = 0X23; /**< Quaternion x Data <15:8>.*/
        const uint8_t QUATERNION_DATA_Y_LSB = 0X24; /**< Quaternion y Data <7:0>.*/
        const uint8_t QUATERNION_DATA_Y_MSB = 0X25; /**< Quaternion y Data <15:8>.*/
        const uint8_t QUATERNION_DATA_Z_LSB = 0X26; /**< Quaternion z Data <7:0>.*/
        const uint8_t QUATERNION_DATA_Z_MSB = 0X27; /**< Quaternion z Data <15:8>.*/

        /* Linear acceleration data registers */
        const uint8_t LINEAR_ACCEL_DATA_X_LSB = 0X28; /**< Linear Acceleration Data X <7:0>.*/
        const uint8_t LINEAR_ACCEL_DATA_X_MSB = 0X29; /**< Linear Acceleration DataX <15:8>.*/
        const uint8_t LINEAR_ACCEL_DATA_Y_LSB = 0X2A; /**< Linear Acceleration Data Y <7:0>.*/
        const uint8_t LINEAR_ACCEL_DATA_Y_MSB = 0X2B; /**< Linear Acceleration Data Y <15:8>.*/
        const uint8_t LINEAR_ACCEL_DATA_Z_LSB = 0X2C; /**< Linear Acceleration Data Z <7:0>.*/
        const uint8_t LINEAR_ACCEL_DATA_Z_MSB = 0X2D; /**< Linear Acceleration Data Z <15:8>.*/

        /* Gravity data registers */
        const uint8_t GRAVITY_DATA_X_LSB = 0X2E; /**< Gravity Vector Data X <7:0>.*/
        const uint8_t GRAVITY_DATA_X_MSB = 0X2F; /**< Gravity Vector Data X <15:8>.*/
        const uint8_t GRAVITY_DATA_Y_LSB = 0X30; /**< Gravity Vector Data Y <7:0>.*/
        const uint8_t GRAVITY_DATA_Y_MSB = 0X31; /**< Gravity Vector Data Y <15:8>.*/
        const uint8_t GRAVITY_DATA_Z_LSB = 0X32; /**< Gravity Vector Data Z <7:0>.*/
        const uint8_t GRAVITY_DATA_Z_MSB = 0X33; /**< Gravity Vector Data Z <15:8>.*/

        /* Temperature data register */
        const uint8_t TEMP_ADDR = 0X34; /**< Temperature.*/

        /* Status registers */
        const uint8_t CALIB_STAT = 0X35; /**< Calibration status.*/
        const uint8_t SELFTEST_RESULT = 0X36; /**< Self test result.*/
        const uint8_t INTR_STAT = 0X37; /**< Status.*/

        const uint8_t SYS_CLK_STAT = 0X38; /**< System clock status.*/
        const uint8_t SYS_STAT = 0X39; /**< System Status Code.*/
        const uint8_t SYS_ERR = 0X3A; /**< System Error Code.*/

        /* Value selection register */
        const uint8_t UNIT_SEL = 0X3B; /**< Unit selection.*/

        /* Mode registers */
        const uint8_t OPR_MODE = 0X3D; /**< Operation mode <3:0>.*/
        const uint8_t PWR_MODE = 0X3E; /**< Power mode <1:0>.*/

        const uint8_t SYS_TRIGGER = 0X3F; /**< System trigger.*/
        const uint8_t TEMP_SOURCE = 0X40; /**< Temperature source <1:0>.*/

        /* Axis remap registers */
        const uint8_t AXIS_MAP_CONFIG = 0X41; /**< Remapped value.*/
        const uint8_t AXIS_MAP_SIGN = 0X42; /**< Remapped sign.*/

        /* Accelerometer Offset registers */
        const uint8_t ACCEL_OFFSET_X_LSB = 0X55; /**< Accelerometer Offset X <7:0>.*/
        const uint8_t ACCEL_OFFSET_X_MSB = 0X56; /**< Accelerometer Offset X <15:8>.*/
        const uint8_t ACCEL_OFFSET_Y_LSB = 0X57; /**< Accelerometer Offset Y <7:0>.*/
        const uint8_t ACCEL_OFFSET_Y_MSB = 0X58; /**< Accelerometer Offset Y <15:8>.*/
        const uint8_t ACCEL_OFFSET_Z_LSB = 0X59; /**< Accelerometer Offset Z <7:0>.*/
        const uint8_t ACCEL_OFFSET_Z_MSB = 0X5A; /**< Accelerometer Offset Z <15:8>.*/

        /* Magnetometer Offset registers */
        const uint8_t MAG_OFFSET_X_LSB = 0X5B; /**< Magnetometer Offset X <7:0>.*/
        const uint8_t MAG_OFFSET_X_MSB = 0X5C; /**< Magnetometer Offset X <15:8>.*/
        const uint8_t MAG_OFFSET_Y_LSB = 0X5D; /**< Magnetometer Offset Y <7:0>.*/
        const uint8_t MAG_OFFSET_Y_MSB = 0X5E; /**< Magnetometer Offset Y <15:8>.*/
        const uint8_t MAG_OFFSET_Z_LSB = 0X5F; /**< Magnetometer Offset Z <7:0>.*/
        const uint8_t MAG_OFFSET_Z_MSB = 0X60; /**< Magnetometer Offset Z <15:8>.*/

        /* Gyroscope Offset register s*/
        const uint8_t GYRO_OFFSET_X_LSB = 0X61; /**< Gyroscope Offset X <7:0>.*/
        const uint8_t GYRO_OFFSET_X_MSB = 0X62; /**< Gyroscope Offset X <15:8>.*/
        const uint8_t GYRO_OFFSET_Y_LSB = 0X63; /**< Gyroscope Offset Y <7:0>.*/
        const uint8_t GYRO_OFFSET_Y_MSB = 0X64; /**< Gyroscope Offset Y <15:8>.*/
        const uint8_t GYRO_OFFSET_Z_LSB = 0X65; /**< Gyroscope Offset Z <7:0>.*/
        const uint8_t GYRO_OFFSET_Z_MSB = 0X66; /**< Gyroscope Offset Z <15:8>.*/

        /* Radius registers */
        const uint8_t ACCEL_RADIUS_LSB = 0X67; /**< Accelerometer Radius LSB.*/
        const uint8_t ACCEL_RADIUS_MSB = 0X68; /**< Accelerometer Radius MSB.*/
        const uint8_t MAG_RADIUS_LSB = 0X69; /**< Magnetometer Radius.*/
        const uint8_t MAG_RADIUS_MSB = 0X6A; /**< Magnetometer Radius.*/
    }
}

// Forward declarations
class I2C;

/**
 * BNO 055 helper class.
 * A class that facilitates interaction with the BNO 055 intelligent absolute orientation sensor produced by Bosch.
 */
class BNO_055
{
public:
    /**
     * Custom constructor.
     * Tries to set up I²C and identify the BNO055.
     *
     * @param address The address of the BNO055 sensor (0x28 or 0x29).
     * @throw BNO_055Exception in case the sensor could not be identified.
     */
    BNO_055(uint8_t address);
    /**
     * Custom destructor.
     * Releases resources.
     */
    ~BNO_055();
    /**
     * Sets the operation mode of the BNO055 sensor.
     *
     * @param mode The mode to be set.
     */
    void setOperationMode(uint8_t mode);
    /**
     * Gets the raw acceleration data.
     *
     * @param buffer A pointer to an array with a size of at least 3.
     * @return True if the reading was successful, false otherwise.
     */
    bool getRawAcceleration(int16_t *buffer);
    /**
     * Gets the raw magnetometer data.
     *
     * @param buffer A pointer to an array with a size of at least 3.
     * @return True if the reading was successful, false otherwise.
     */
    bool getRawMagnetometer(int16_t *buffer);
    /**
     * Gets the raw gyroscope data.
     *
     * @param buffer A pointer to an with a size of at least 3.
     * @return True if the reading was successful, false otherwise.
     */
    bool getRawGyroscope(int16_t *buffer);
    /**
     * Gets euler fusion data.
     *
     * @param buffer A pointer to an array with a size of at least 3.
     * @return True if the reading was successful, false otherwise.
     */
    bool getEuler(float *buffer);

private:
    std::unique_ptr<I2C> i2c;
};

#endif //OBJECT_HUNT_BASE_BNO_055_H
