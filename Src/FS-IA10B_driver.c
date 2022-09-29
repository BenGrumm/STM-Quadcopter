#include "FS-IA10B_driver.h"

uint16_t fsia10b_channel_values[FSIA10B_NUMBER_CHANNELS] = {0};
static uint8_t currentPos = 0;

/**
 * @brief Funtion to call in interrupt handler to calculate the length of each channel
 * 
 * @param htim handle
 */
void FSIA10B_INT(TIM_HandleTypeDef *htim){
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        // 1 tick = 1uS
        // Normal length from 1ms - 2ms, over 2ms is gap between last and first signal
        // (min of the inteval signal is 4ms)
        if(val >= FSIA10B_TIME_START_GREATER || currentPos == FSIA10B_NUMBER_CHANNELS){
            currentPos = 0;
        }else{
            fsia10b_channel_values[currentPos++] = val;
        }

        // Reset
        __HAL_TIM_SET_COUNTER(htim, 0);
    }
}