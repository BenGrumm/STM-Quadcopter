#ifndef ESC_CONTROLLER_HEADER
#define ESC_CONTROLLER_HEADER

#include "stm32f4xx_hal.h"

// https://img.banggood.com/file/products/20171201012630BLHeli_32%20manual%20ARM%20Rev32.x.pdf
// Arming sequence P5
// Beeps - Throttle calibration P8

void ESC_setup(TIM_HandleTypeDef* pwm_tim);
void ESC_arm(TIM_HandleTypeDef* pwm_tim);
void ESC_throttleCalibration(TIM_HandleTypeDef* pwm_tim);

#endif