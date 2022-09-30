#include "MPU6050.h"

// Functions only used in this file
static void calculateOrientationFusion(MPU6050* device, float* gyroValues, float* accelValues);
static void MPU6050_convertRegsToSignedVals(uint8_t* regs, int16_t* dest, uint8_t numRegs);
static void convertGyroRegsToDegreesS(MPU6050* device, int16_t* gyroValues, float* values);
static void convertAccelRegToGs(MPU6050* device, int16_t* accelValues, float* values);
static void convertTemperatureRegToCelsius(int16_t* regValue, float* actualVal);
static void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance);

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

    // Setup dma vars
    mpu->hasNewData = false;
    for(int i = 0; i < MPU6050_CONSECUTIVE_DATA_REGS; i++){
        mpu->dmaDataBuffer[i] = 0;
    }
    mpu->currentGyroReadTime = 0;

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
            I2C1_ClearBusyFlagErratum(mpu->i2c_handler);
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
 * @brief Function to clear busy when the external device holds SDA low 
 * (https://electronics.stackexchange.com/questions/351972/hal-i2c-hangs-cannot-be-solved-with-standard-routine-use-to-unlock-i2c)
 * 
 * @param instance i2c instance to clear bus of
 */
static void I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int timeout =100;
    int timeout_cnt=0;

    // 1. Clear PE bit.
    instance->Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Alternate    = GPIO_AF4_I2C3;
    GPIO_InitStruct.Pull         = GPIO_PULLUP;
    GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin          = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin          = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);


    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);

    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);

    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    instance->Instance->CR1 |= 0x0001;

    // Call initialization function.
    HAL_I2C_Init(instance);
}

/**
 * @brief Function to read the accel, temperature and gyro data using a DMA
 * 
 * @param device the device to read the data from
 * @return HAL_StatusTypeDef succesful I2C mem read
 */
HAL_StatusTypeDef MPU6050_ReadDataDMA(MPU6050* device){
    device->currentGyroReadTime = HAL_GetTick();
    return HAL_I2C_Mem_Read_DMA(device->i2c_handler, MPU6050_I2C_ADDR, MPU_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, device->dmaDataBuffer, MPU6050_CONSECUTIVE_DATA_REGS);
}

/**
 * @brief Function that should be called in the DMA callback
 * 
 * @param device the device that is being read
 */
void MPU6050_DMAReadCplt(MPU6050* device){
    device->hasNewData = true;
}

/**
 * @brief When using DMA this function should be called in the main loop of the function
 * 
 * @param device the device that is being read
 */
void MPU6050_DMALoop(MPU6050* device){
    if(device->hasNewData){
        int16_t convertedRegs[MPU6050_CONSECUTIVE_DATA_REGS / 2];
        MPU6050_convertRegsToSignedVals(device->dmaDataBuffer, convertedRegs, MPU6050_CONSECUTIVE_DATA_REGS);

        float gyroValues[3], temperatureVal, accelValues[3];

        // Convert all register values to their actual values
        // 0-2 contain accel - 3 contains temp - 4-6 contains gyro
        convertGyroRegsToDegreesS(device, &convertedRegs[4], gyroValues);
        convertAccelRegToGs(device, convertedRegs, accelValues);
        convertTemperatureRegToCelsius(&convertedRegs[3], &temperatureVal);

        calculateOrientationFusion(device, gyroValues, accelValues);
        
        device->hasNewData = false;

        // Make sure I2C is free then start DMA read again
        while(HAL_I2C_GetState(device->i2c_handler) != HAL_I2C_STATE_READY);
        MPU6050_ReadDataDMA(device);
    }
}

/**
 * @brief function to take the gyro and accel values and use the Fusion library to calculate orientation
 * 
 * @param device device that the data was read from
 * @param gyroValues gyro values that are read from MPU and converted to degrees/s
 * @param accelValues gyro values that are read from MPU and converted to g
 */
static void calculateOrientationFusion(MPU6050* device, float* gyroValues, float* accelValues){
    float elapsedTime = (device->currentGyroReadTime - device->lastGyroReadingTime) / 1000.0f;

    // Calculate position 
    const FusionVector gyroscope = {{gyroValues[0], gyroValues[1], gyroValues[2]}};
    const FusionVector accelerometer = {{accelValues[0], accelValues[1], accelValues[2]}};
    FusionAhrsUpdateNoMagnetometer(device->ahrs, gyroscope, accelerometer, elapsedTime);

    device->lastGyroReadingTime = device->currentGyroReadTime;
}

/**
 * @brief Convert an array of unsigned 8 bit numbers retreived from the MPU to the signed 16 bit values
 * 
 * @param regs array of unsigned 8 bit values 
 * @param dest where to store the signed 16 bit values (must be half the size of the number of 8 bit)
 * @param numRegs number of 8 bit regs passed (must be a multiple of 2)
 */
static void MPU6050_convertRegsToSignedVals(uint8_t* regs, int16_t* dest, uint8_t numRegs){

    if(numRegs % 2 != 0){
        return;
    }

    for(int i = 0; i < numRegs; i+=2){
        dest[i / 2] = ((int16_t) regs[i]) << 8;
        dest[i / 2] |= regs[i + 1];
    }
}

/**
 * @brief Converts the signed 16 bit gyro values from the MPU to degrees per second using set sensitivity
 * 
 * @param device device the values were retreived from
 * @param regValues values retreived from device (must be of size 3 with X, Y, Z values)
 * @param converted where to store converted values (must be of size 3 stored in X, Y, Z order)
 */
static void convertGyroRegsToDegreesS(MPU6050* device, int16_t* regValues, float* converted){
    for(int i = 0; i < 3; i++){
        switch(device->MPU_Gyro_Range){
            case MPU_GYRO_SCALE_RANGE_250:
                converted[i] = (float) regValues[i] / MPU_GYRO_SENSITIVITY_250;
                break;
            case MPU_GYRO_SCALE_RANGE_500:
                converted[i] = (float) regValues[i] / MPU_GYRO_SENSITIVITY_500;
                break;
            case MPU_GYRO_SCALE_RANGE_1000:
                converted[i] = (float) regValues[i] / MPU_GYRO_SENSITIVITY_1000;
                break;
            case MPU_GYRO_SCALE_RANGE_2000:
                converted[i] = (float) regValues[i] / MPU_GYRO_SENSITIVITY_2000;
                break;
        }
    }
}

/**
 * @brief Converts the signed 16 bit values to the g value depending on sensitivity set
 * 
 * @param device mpu values were retrieved from 
 * @param regValues 16 bit values retrieved from the mpu
 * @param converted destination for the converted values
 */
static void convertAccelRegToGs(MPU6050* device, int16_t* regValues, float* converted){
    for(int i = 0; i < 3; i++){
        switch(device->MPU_Accel_Range){
            case MPU_ACCEL_SCALE_RANGE_2G:
                converted[i] = (float) regValues[i] / MPU_ACCEL_SENSITIVITY_2G;
                break;
            case MPU_ACCEL_SCALE_RANGE_4G:
                converted[i] = (float) regValues[i] / MPU_ACCEL_SENSITIVITY_4G;
                break;
            case MPU_ACCEL_SCALE_RANGE_8G:
                converted[i] = (float) regValues[i] / MPU_ACCEL_SENSITIVITY_8G;
                break;
            case MPU_ACCEL_SCALE_RANGE_16G:
                converted[i] = (float) regValues[i] / MPU_ACCEL_SENSITIVITY_16G;
                break;
        }
    }
}

/**
 * @brief Converts the signed value stored in the MPU register to celsius
 * 
 * @param regValue the signed 16 bit int retreived from the register
 * @param actualVal location to store value in celsius
 */
static void convertTemperatureRegToCelsius(int16_t* regValue, float* actualVal){
    *actualVal = (*regValue / 340.0) + 36.53;
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