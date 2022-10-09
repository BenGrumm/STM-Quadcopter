#include "ESCController.h"

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

    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_4, 0);

    return HAL_OK;
}

void ESC_arm(ESC_4Channels* escs){
    HAL_Delay(2000);

    uint32_t quarterThrottle = escs->min_throttle + (escs->max_throttle - escs->min_throttle) / 4;

    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_1, quarterThrottle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_2, quarterThrottle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_3, quarterThrottle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_4, quarterThrottle);

    HAL_Delay(2000);

    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_1, escs->min_throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_2, escs->min_throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_3, escs->min_throttle);
    __HAL_TIM_SET_COMPARE(escs->pwm_tim, TIM_CHANNEL_4, escs->min_throttle);
}

void ESC_throttleCalibration(TIM_HandleTypeDef* pwm_tim){

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