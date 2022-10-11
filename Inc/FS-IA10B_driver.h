#ifndef FSIA10B_DRIVER_H
#define FSIA10B_DRIVER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define FREQ_APB1       96000000 // 96MHz
#define TIM_PRESCALER   96     // 96000000 / 96 = 1MHz = 0.01uS

#define FSIA10B_NUMBER_CHANNELS         8
#define FSIA10B_TIME_CHANNEL_MIN_MICRO  1000        // The minimum PPM signal length in uS (micros seconds)
#define FSIA10B_TIME_CHANNEL_MAX_MICRO  2000        // The maximum PPM signal length in uS (micros seconds)
#define FSIA10B_TIME_START_BUFFER       500
#define FSIA10B_TIME_START_GREATER      (FSIA10B_TIME_CHANNEL_MAX_MICRO + FSIA10B_TIME_START_BUFFER)
#define FSIA10B_RECEIVER_TIMEOUT_MS     1000

#define FSIA10B_CHANNEL_PITCH       0
#define FSIA10B_CHANNEL_ROLL        1
#define FSIA10B_CHANNEL_THROTTLE    2
#define FSIA10B_CHANNEL_YAW         3
#define FSIA10B_CHANNEL_SWA         4
#define FSIA10B_CHANNEL_SWB         5
#define FSIA10B_CHANNEL_SWC         6
#define FSIA10B_CHANNEL_SWD         7

#define FSIA10B_CHANNEL_ARM         FSIA10B_CHANNEL_SWA

typedef struct {
    TIM_HandleTypeDef *htim;
    volatile uint16_t channels[FSIA10B_NUMBER_CHANNELS];
    uint8_t currentPos;
    uint32_t lastUpdate;
} FSIA10B;

void FSIA10B_setup(FSIA10B* receiver);
void FSIA10B_INT(FSIA10B *receiver, TIM_HandleTypeDef *htim);
uint8_t FSIA10B_isReceivingSignal(FSIA10B *receiver);
uint8_t FSIA10B_isArmed(FSIA10B *receiver);
uint8_t FSIA10B_throttleIsOff(FSIA10B *receiver);

#endif