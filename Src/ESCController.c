#include "ESCController.h"

static void ESC_startAll(ESC_4Channels* escs);
static void ESC_stopAll(ESC_4Channels* escs);
static void ESC_armStageOne(ESC_4Channels* escs);
static void ESC_armStageTwo(ESC_4Channels* escs);
static void ESC_armStageThree(ESC_4Channels* escs);

/**
 * @brief 
 * 
 * @param escs 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef ESC_setup(ESC_4Channels* escs){
    if(escs->pwm_tim == NULL || escs->max_throttle - escs->min_throttle <= 0){
        return HAL_ERROR;
    }

    escs->currentState = INITIALISE;

    ESC_writeAll(escs, 0);

    return HAL_OK;
}

/**
 * @brief Function to arms the escs in a blocking manor
 * 
 * @param escs The escs to arm
 */
void ESC_arm(ESC_4Channels* escs){
    ESC_armStageOne(escs);

    HAL_Delay(ESC_ARM_STAGE_ONE_DELAY);

    ESC_armStageTwo(escs);

    HAL_Delay(ESC_ARM_STAGE_TWO_DELAY);

    ESC_armStageThree(escs);
}

/**
 * @brief Function to arm the drone in a non-block manor by calling the function in a loop
 * 
 * @param escs The escs that are being armed
 * @return uint8_t 0 (false) if the escs havn't been armed 1 (true) if they have been succesfully armed
 */
uint8_t ESC_armNonBlocking(ESC_4Channels* escs){
    if(escs->currentState == INITIALISE){
        ESC_armStageOne(escs);
        escs->startTime = HAL_GetTick();
        escs->currentState = LOW_THROTTLE;
        return 0;
    }

    if(escs->currentState == LOW_THROTTLE){
        if((HAL_GetTick() - escs->startTime) >= ESC_ARM_STAGE_ONE_DELAY){
            ESC_armStageTwo(escs);
            escs->startTime = HAL_GetTick();
            escs->currentState = QUARTER_THROTTLE;
        }
        return 0;
    }

    if(escs->currentState == QUARTER_THROTTLE){
        if((HAL_GetTick() - escs->startTime) >= ESC_ARM_STAGE_TWO_DELAY){
            ESC_armStageThree(escs);
            // Reset start state and return true as finished
            escs->currentState = INITIALISE;
            return 1;
        }
        return 0;
    }

    return 0;
}

/**
 * @brief Function to start the arming of the escs by turning on pwm and setting min throttle
 * 
 * @param escs The escs to start to arm
 */
static void ESC_armStageOne(ESC_4Channels* escs){
    // Make sure all stopped
    ESC_stopAll(escs);

    ESC_writeAll(escs, escs->min_throttle);

    // Setup PWM for motors
    ESC_startAll(escs);
}

/**
 * @brief Function to continue the arming of the escs by increasing the throttle
 * 
 * @param escs The escs that are being armed
 */
static void ESC_armStageTwo(ESC_4Channels* escs){
    uint32_t quarterThrottle = escs->min_throttle + ((escs->max_throttle - escs->min_throttle) / 4);

    ESC_writeAll(escs, quarterThrottle);
}

/**
 * @brief Function to finalise the startup process of the escs by setting the throttle back to min
 * 
 * @param escs The escs that are being armed
 */
static void ESC_armStageThree(ESC_4Channels* escs){
    ESC_writeAll(escs, escs->min_throttle);
}

void ESC_disarm(ESC_4Channels* escs){
    ESC_stopAll(escs);
    ESC_writeAll(escs, 0);
}

/**
 * @brief NOT WORKING
 * 
 * @param escs 
 */
void ESC_throttleCalibration(ESC_4Channels* escs){
    // Make sure all stopped
    ESC_stopAll(escs);

    ESC_writeAll(escs, escs->max_throttle);

    // Setup PWM for motors
    ESC_startAll(escs);

    HAL_Delay(3500);

    ESC_writeAll(escs, escs->min_throttle);

    HAL_Delay(3200);
}

/**
 * @brief Function to write values to the individual escs
 * 
 * @param escs The escs controlling the motors
 * @param frontLeft The value to set the front left motor to
 * @param backRight The value to set the back right motor to
 * @param frontRight The value to set the front right motor to
 * @param backLeft The value to set the back left motor to
 */
void ESC_writeMotors(ESC_4Channels* escs, uint32_t frontLeft, uint32_t backRight, uint32_t frontRight, uint32_t backLeft){
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_1, frontLeft);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_2, backRight);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_3, frontRight);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_4, backLeft);
}

/**
 * @brief Function to write a value to the compare register of all Escs (pwm channels)
 * 
 * @param escs The escs to write to
 * @param throttle The throttle to sett all of the escs to
 */
void ESC_writeAll(ESC_4Channels* escs, uint32_t throttle){
    ESC_writeMotors(escs, throttle, throttle, throttle, throttle);
}

/**
 * @brief Function to start the pwm signals to all escs
 * 
 * @param escs The escs to set up
 */
static void ESC_startAll(ESC_4Channels* escs){
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_4);
}

/**
 * @brief Function to stop the pwm signals to all escs
 * 
 * @param escs The escs to stop
 */
static void ESC_stopAll(ESC_4Channels* escs){
    HAL_TIM_PWM_Stop(escs->pwm_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(escs->pwm_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(escs->pwm_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(escs->pwm_tim, TIM_CHANNEL_4);
}

/**
 * @brief Map function to map value from one range into another range (taken from arduino)
 * 
 * @param x value you want to map
 * @param in_min min of the range containing the input value
 * @param in_max max of the range containing the input value
 * @param out_min min of the range containing the output value
 * @param out_max max of the range containing the output value
 * @return long new value withing output range
 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}