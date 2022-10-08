#include "ESCController.h"

/**
 * @brief 
 * 
 * @param pwm_tim 
 */
void ESC_setup(TIM_HandleTypeDef* pwm_tim){
    __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_4, 0);
}

void ESC_arm(TIM_HandleTypeDef* pwm_tim){

}

void ESC_throttleCalibration(TIM_HandleTypeDef* pwm_tim){

}