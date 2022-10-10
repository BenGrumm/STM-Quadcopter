#include "ESCController.h"

static void ESC_startAll(ESC_4Channels* escs);
static void ESC_stopAll(ESC_4Channels* escs);

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

    ESC_writeAll(escs, 0);

    return HAL_OK;
}

void ESC_arm(ESC_4Channels* escs){
    // Make sure all stopped
    ESC_stopAll(escs);

    ESC_writeAll(escs, escs->min_throttle);

    // Setup PWM for motors
    ESC_startAll(escs);

    HAL_Delay(500);

    uint32_t quarterThrottle = escs->min_throttle + (escs->max_throttle - escs->min_throttle) / 4;

    ESC_writeAll(escs, quarterThrottle);

    HAL_Delay(100);

    ESC_writeAll(escs, escs->min_throttle);
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

void ESC_writeAll(ESC_4Channels* escs, uint32_t throttle){
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_1, throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_2, throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_3, throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_4, throttle);
}

static void ESC_startAll(ESC_4Channels* escs){
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(escs->pwm_tim, TIM_CHANNEL_4);
}

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