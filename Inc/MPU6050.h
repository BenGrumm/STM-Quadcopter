/**
 * @file MPU6050.h
 * @author Ben Grummitt
 * @brief Simple driver to interface with MPU6050 accelerometer and gyroscope
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * https://forum.arduino.cc/t/i2c-protocol-tutorial-using-an-mpu6050/387512
 * https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 * https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
 *
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * P29 For Accel Regs
 * P31 For Gryo Regs
 * P40 For PWR_MNGMT regs
 * 
 */
#ifndef MPU6050_I2C_DRIVER_H
#define MPU6050_I2C_DRIVER_H

#include "stm32f4xx_hal.h"
#include "Fusion.h"
#include <math.h>

/**
 * Defines
 */
#define MPU6050_I2C_ADDR    (0x68 << 1)

// If we read 14 regs from MPU_ACCEL_XOUT_H we get all ACCELL then all temp
// then all the gyro regs
#define MPU6050_CONSECUTIVE_DATA_REGS 14

/**
 * MPU Registers
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#define MPU_GYRO_CONFIG         0x1B
#define MPU_ACCEL_CONFIG        0x1C

#define MPU_ACCEL_XOUT_H        0x3B
#define MPU_ACCEL_XOUT_L        0x3C
#define MPU_ACCEL_YOUT_H        0x3D
#define MPU_ACCEL_YOUT_L        0x3E
#define MPU_ACCEL_ZOUT_H        0x3F
#define MPU_ACCEL_ZOUT_L        0x40

#define MPU_GYRO_XOUT_H         0x43
#define MPU_GYRO_XOUT_L         0x44
#define MPU_GYRO_YOUT_H         0x45
#define MPU_GYRO_YOUT_L         0x46
#define MPU_GYRO_ZOUT_H         0x47
#define MPU_GYRO_ZOUT_L         0x48

#define MPU_SIGNAL_PATH_RESET   0x68

#define MPU_PWR_MGMT_1          0x6B

#define MPU_PWR_MGMT_2          0x6C


/**
 * Accelerometer Full Scale Range
 */
#define MPU_ACCEL_SCALE_RANGE_2G    0
#define MPU_ACCEL_SCALE_RANGE_4G    1
#define MPU_ACCEL_SCALE_RANGE_8G    2
#define MPU_ACCEL_SCALE_RANGE_16G   3

/**
 * Accelerometer LSB Sensitivity
 */
#define MPU_ACCEL_SENSITIVITY_2G    16384.0
#define MPU_ACCEL_SENSITIVITY_4G    8192.0
#define MPU_ACCEL_SENSITIVITY_8G    4096.0
#define MPU_ACCEL_SENSITIVITY_16G   2048.0

/**
 * Gyro Full Scale Range
 */
#define MPU_GYRO_SCALE_RANGE_250    0
#define MPU_GYRO_SCALE_RANGE_500    1
#define MPU_GYRO_SCALE_RANGE_1000   2
#define MPU_GYRO_SCALE_RANGE_2000   3

/**
 * Gyro LSB Sensitivity
 */
#define MPU_GYRO_SENSITIVITY_250    131.0
#define MPU_GYRO_SENSITIVITY_500    65.5
#define MPU_GYRO_SENSITIVITY_1000   32.8
#define MPU_GYRO_SENSITIVITY_2000   16.4

/**
 * MPU Struct
 */
typedef struct {
    I2C_HandleTypeDef* i2c_handler;

    uint8_t MPU_Gyro_Range; // One of MPU_GYRO_SCALE_RANGE_250 - MPU_GYRO_SCALE_RANGE_2000
    uint8_t MPU_Accel_Range; //  One of MPU_ACCEL_SCALE_RANGE_2G - MPU_ACCEL_SCALE_RANGE_16G

    FusionAhrs* ahrs;
    float gyro_angle[3]; // X, Y, Z
    float accel_angle[3]; // X, Y, Z

    float position[3]; // Combined gyro and accel angles, X, Y, Z - Roll, Pitch, Yaw

    // Used for dma
    uint32_t currentGyroReadTime;
    bool hasNewData;
    uint8_t dmaDataBuffer[MPU6050_CONSECUTIVE_DATA_REGS];

    uint32_t lastGyroReadingTime; // stores time to calculate gyro movement
}MPU6050;

uint8_t setupMPU6050(MPU6050* mpu, I2C_HandleTypeDef* i2c_handler, FusionAhrs* ahrs);

HAL_StatusTypeDef MPU6050_ReadDataDMA(MPU6050* device);
void MPU6050_DMAReadCplt(MPU6050* device);
void MPU6050_DMALoop(MPU6050* device);

HAL_StatusTypeDef MPU6050_calculateGyroAndMPUError(MPU6050* device, float* gyroError, float* accelError);

HAL_StatusTypeDef MPU6050_readMPUAndCalculatePositionFusion(MPU6050* device);
HAL_StatusTypeDef MPU6050_readMPUAndCalculatePosition(MPU6050* device);

HAL_StatusTypeDef MPU6050_readGyro(MPU6050* device, float* values);
HAL_StatusTypeDef MPU6050_readAccelerometer(MPU6050* device, float* values);

HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen);
HAL_StatusTypeDef MPU6050_writeRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen);

#endif