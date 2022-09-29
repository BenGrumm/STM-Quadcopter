#ifndef FSIA10B_DRIVER_H
#define FSIA10B_DRIVER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define FREQ_APB1       96000000 // 96MHz
#define TIM_PRESCALER   96     // 96000000 / 96 = 1MHz = 0.01uS

#define FSIA10B_NUMBER_CHANNELS         8
#define FSIA10B_TIME_CHANNEL_MAX_MICRO  2000
#define FSIA10B_TIME_START_BUFFER       500
#define FSIA10B_TIME_START_GREATER      (FSIA10B_TIME_CHANNEL_MAX_MICRO + FSIA10B_TIME_START_BUFFER)

extern uint16_t fsia10b_channel_values[FSIA10B_NUMBER_CHANNELS];

void FSIA10B_INT(TIM_HandleTypeDef *htim);

#endif