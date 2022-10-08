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
#define FSIA10B_RECEIVER_TIMEOUT        1000

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t channels[FSIA10B_NUMBER_CHANNELS];
    uint8_t currentPos;
    uint32_t lastUpdate;
} FSIA10B;

void FSIA10B_setup(FSIA10B* receiver);
void FSIA10B_INT(FSIA10B *receiver);

#endif