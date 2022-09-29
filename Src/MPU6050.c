#include "MPU6050.h"

/**
 * @brief MPU setup function that will initialise MPU struct and setup the MPU6050 for use
 * 
 * @param mpu struct to store data
 * @param i2c_handler handler of the I2C bus to use
 * @return uint8_t 0 if no errors else the amount of errors / timeouts / busy
 */
uint8_t setupMPU6050(MPU6050* mpu, I2C_HandleTypeDef* i2c_handler, FusionAhrs* ahrs){
    int returnError = 0;

    // Initialise the mpu struct
    mpu->i2c_handler = i2c_handler;
    mpu->ahrs = ahrs;

    FusionAhrsInitialise(ahrs);

    for(int i = 0; i < 3; i++){
        mpu->accel_angle[i] = 0;
        mpu->gyro_angle[i] = 0;
        mpu->position[i] = 0;
    }

    mpu->lastGyroReadingTime = HAL_GetTick();

    // MPU-6050 addr = 0x68
    uint8_t tempReg = 0;

    // Configure power settings
    tempReg = 0;
    HAL_StatusTypeDef error;
    int i = 0;

    do{
        error = MPU6050_writeRegisters(mpu, MPU_PWR_MGMT_1, &tempReg, 1);

        if(error == HAL_BUSY){
            // Apply fix ???
        }
    } while(i++ < 3 && error == HAL_BUSY);

    returnError += (error != HAL_OK);

    // Configure gyro FS_SEL for scale range
    tempReg = 0 | (mpu->MPU_Gyro_Range << 3);
    error = MPU6050_writeRegisters(mpu, MPU_GYRO_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    // Configure Accell settings
    tempReg = 0 | (mpu->MPU_Accel_Range << 3);
    error = MPU6050_writeRegisters(mpu, MPU_ACCEL_CONFIG, &tempReg, 1);

    returnError += (error != HAL_OK);

    return returnError;
}

/**
 * @brief Function to read the MPU (gyro & accelerometer) then use the fusion library by
 * xioTechnologies (https://github.com/xioTechnologies/Fusion) to calculate the position
 * 
 * @param device device which we read the data from
 * @return HAL_StatusTypeDef errors during I2C communication
 */
HAL_StatusTypeDef MPU6050_readMPUAndCalculatePositionFusion(MPU6050* device){
    float gyroTemp[3], accelTemp[3];

    // Need time in seconds between readings for degrees per second gyro output
    uint32_t previouseGyroTime = device->lastGyroReadingTime;
    device->lastGyroReadingTime = HAL_GetTick();
    float elapsedTime = (device->lastGyroReadingTime - previouseGyroTime) / 1000.0f; // Calculate in seconds
    
    HAL_StatusTypeDef error = MPU6050_readGyro(device, gyroTemp);

    if(error == HAL_OK){
        error = MPU6050_readAccelerometer(device, accelTemp);

        // Calculate position 
        const FusionVector gyroscope = {{gyroTemp[0], gyroTemp[1], gyroTemp[2]}}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {{accelTemp[0], accelTemp[1], accelTemp[2]}}; // replace this with actual accelerometer data in g
        FusionAhrsUpdateNoMagnetometer(device->ahrs, gyroscope, accelerometer, elapsedTime);
    }

    return error; 
}

/**
 * @brief Function to read the MPU (gyro & accelerometer) then use trig and integration to find position
 * 
 * @param device device which we read the data from
 * @return HAL_StatusTypeDef errors during I2C communication 
 */
HAL_StatusTypeDef MPU6050_readMPUAndCalculatePosition(MPU6050* device){
    float gyroTemp[3], accelTemp[3];

    uint32_t previouseGyroTime = device->lastGyroReadingTime;
    device->lastGyroReadingTime = HAL_GetTick();
    float elapsedTime = (device->lastGyroReadingTime - previouseGyroTime) / 1000.0; // Calculate in seconds

    HAL_StatusTypeDef error = MPU6050_readGyro(device, gyroTemp);

    if(error == HAL_OK){
        // Account for errors calculated
        gyroTemp[0] += 2.93;
        gyroTemp[1] -= 1.27;
        gyroTemp[2] += 1.45;
        device->gyro_angle[0] = device->gyro_angle[0] + (gyroTemp[0] * elapsedTime);
        device->gyro_angle[1] = device->gyro_angle[1] + (gyroTemp[1] * elapsedTime);
        device->gyro_angle[2] = device->gyro_angle[2] + (gyroTemp[2] * elapsedTime);
        error = MPU6050_readAccelerometer(device, accelTemp);
    }

    if(error == HAL_OK){
        device->accel_angle[0] = (atan(accelTemp[1] / sqrt(pow(accelTemp[0], 2) + pow(accelTemp[2], 2))) * 180 / M_PI) + 0.32; // Calculate accelerometer X and account for error
        device->accel_angle[1] = (atan(-1 * accelTemp[0] / sqrt(pow(accelTemp[1], 2) + pow(accelTemp[2], 2))) * 180 / M_PI) - 0.4; // Calculate accelerometer Y and account for error

        device->gyro_angle[0] = 0.96 * device->gyro_angle[0] + 0.04 * device->accel_angle[0];
        device->gyro_angle[1] = 0.96 * device->gyro_angle[1] + 0.04 * device->accel_angle[1];

        device->position[0] = device->gyro_angle[0];
        device->position[1] = device->gyro_angle[1];
        device->position[2] = device->gyro_angle[2];
    }

    return error;
}

/**
 * @brief Function to calculate the error by reading values when the sensor is in stationory home position
 * 
 * @param device mpu to measure error of
 * @param gyroError an array of 3 float values to store the gyro error
 * @param accelError an array of 3 float values to store the accelerometer error
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050_calculateGyroAndMPUError(MPU6050* device, float* gyroError, float* accelError){
    float valuesTemp[3];
    gyroError[0] = 0;
    gyroError[1] = 0;
    gyroError[2] = 0;
    accelError[0] = 0;
    accelError[1] = 0;
    accelError[2] = 0;

    HAL_StatusTypeDef errorGyro, errorAccel;

    for(uint8_t i = 0; i < 200; i++){
        errorGyro = MPU6050_readGyro(device, valuesTemp);

        gyroError[0] += valuesTemp[0];
        gyroError[1] += valuesTemp[1];
        gyroError[2] += valuesTemp[2];
    }

    gyroError[0] /= 200.0;
    gyroError[1] /= 200.0;
    gyroError[2] /= 200.0;

    for(uint8_t i = 0; i < 200; i++){
        errorAccel = MPU6050_readAccelerometer(device, valuesTemp);

        accelError[0] += ((atan(valuesTemp[1] / sqrt(pow(valuesTemp[0], 2) + pow(valuesTemp[2], 2))) * 180 / M_PI));
        accelError[1] += ((atan(-1 * (valuesTemp[0]) / sqrt(pow((valuesTemp[1]), 2) + pow((valuesTemp[2]), 2))) * 180 / M_PI));
        accelError[2] += valuesTemp[2];
    }

    accelError[0] /= 200.0;
    accelError[1] /= 200.0;
    accelError[2] /= 200.0;

    return errorAccel | errorGyro;
}

/**
 * @brief Function to convert 8 bit unsigned int values retrived from slave into the intended signed 16 bit values
 * 
 * @param values values that need to be converted 
 * @param output where to store the converted values
 * @param len the length of the input values array and double the length of the output array
 */
void convertUnsigned8ToSigned16(uint8_t* values, int16_t* output, uint8_t len){
    for(int i = 0; i < len; i+=2){
        output[i / 2] = ((int16_t) values[i]) << 8;
        output[i / 2] |= values[i + 1];
    }
}

/**
 * @brief Function to read 6 consecutive registers in the MPU6050 which is the number needed
 * by the gyro and accelerometer as the store upper and lower values for their 3 axis
 * 
 * @param device the device to read from
 * @param addr address of the first of the six consecutive registers
 * @param output pointer to int16_t array of size 6
 * @return HAL_StatusTypeDef error
 */
HAL_StatusTypeDef MPU6050_readAndConvert6Regs(MPU6050* device, uint8_t addr, int16_t* output){
    uint8_t regValues[6];
    HAL_StatusTypeDef returnVal = MPU6050_readRegisters(device, addr, regValues, 6);

    if(returnVal == HAL_OK){
        convertUnsigned8ToSigned16(regValues, output, 6);
    }
    
    return returnVal;
}

/**
 * @brief Function to read all gyro registers and convert the values using setup parameters
 * 
 * @param device to read data from
 * @param values pointer to float array of size 3
 * @return HAL_StatusTypeDef error in I2C communication
 */
HAL_StatusTypeDef MPU6050_readGyro(MPU6050* device, float* values){
    int16_t gyroValues[3];
    // Read from MPU_GYRO_XOUT_H - MPU_GYRO_ZOUT_L (6 8 bit registers) with high / low regs for HYZ
    HAL_StatusTypeDef error = MPU6050_readAndConvert6Regs(device, MPU_GYRO_XOUT_H, gyroValues);

    if(error == HAL_OK){
        for(int i = 0; i < 3; i++){
            switch(device->MPU_Gyro_Range){
                case MPU_GYRO_SCALE_RANGE_250:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_250;
                    break;
                case MPU_GYRO_SCALE_RANGE_500:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_500;
                    break;
                case MPU_GYRO_SCALE_RANGE_1000:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_1000;
                    break;
                case MPU_GYRO_SCALE_RANGE_2000:
                    values[i] = (float) gyroValues[i] / MPU_GYRO_SENSITIVITY_2000;
                    break;
                default:
                    error = HAL_ERROR;
            }
        }
    }

    return error;
}

/**
 * @brief Function to read all accelerometer registers and convert the values using setup parameters
 * 
 * @param device to read data from
 * @param values pointer to float array of size 3
 * @return HAL_StatusTypeDef error in I2C communication
 */
HAL_StatusTypeDef MPU6050_readAccelerometer(MPU6050* device, float* values){
    int16_t retrivedValues[3];

    // Read from MPU_ACCEL_XOUT_H - MPU_ACCEL_ZOUT_L (6 8 bit registers) with high / low regs for X,Y,Z
    HAL_StatusTypeDef error = MPU6050_readAndConvert6Regs(device, MPU_ACCEL_XOUT_H, retrivedValues);

    if(error == HAL_OK){
        for(int i = 0; i < 3; i++){
            switch(device->MPU_Accel_Range){
                case MPU_ACCEL_SCALE_RANGE_2G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_2G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_4G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_4G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_8G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_8G;
                    break;
                case MPU_ACCEL_SCALE_RANGE_16G:
                    values[i] = (float) retrivedValues[i] / MPU_ACCEL_SENSITIVITY_16G;
                    break;
                default:
                    error = HAL_ERROR;
            }
        }
    }

    return error;
}

/**
 * @brief Function to read registers in the MPU
 * 
 * @param device mpu device with I2C to read from
 * @param reg initial register address to read from
 * @param data array to store retreived data from
 * @param dataLen length of the array and amount of data to read from MPU
 * @return HAL_StatusTypeDef I2C errors
 */
HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Read(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}

/**
 * @brief Function to write to registers in the MPU
 * 
 * @param device mpu device with I2C to write to
 * @param reg initial register address to write to
 * @param data array with data to write
 * @param dataLen length of the array to write to registers
 * @return HAL_StatusTypeDef I2C errors
 */
HAL_StatusTypeDef MPU6050_writeRegisters(MPU6050* device, uint8_t reg, uint8_t* data, uint8_t dataLen){
    return HAL_I2C_Mem_Write(device->i2c_handler, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, dataLen, HAL_MAX_DELAY);
}