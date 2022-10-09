#ifndef ESC_CONTROLLER_HEADER
#define ESC_CONTROLLER_HEADER

#include "stm32f4xx_hal.h"

// https://img.banggood.com/file/products/20171201012630BLHeli_32%20manual%20ARM%20Rev32.x.pdf
// Arming sequence P5
// Beeps - Throttle calibration P8

typedef struct{
    TIM_HandleTypeDef* pwm_tim;
    uint32_t max_throttle;
    uint32_t min_throttle;
}ESC_4Channels;

HAL_StatusTypeDef ESC_setup(ESC_4Channels* escs);
void ESC_arm(ESC_4Channels* escs);
void ESC_throttleCalibration(TIM_HandleTypeDef* pwm_tim);
long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif