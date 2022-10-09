#include "FS-IA10B_driver.h"

/**
 * @brief Function to set up the FSIA10B struct with initialised values
 * 
 * @param receiver struct to set up
 */
void FSIA10B_setup(FSIA10B* receiver){
    for(int i = 0; i < FSIA10B_NUMBER_CHANNELS; i++){
        receiver->channels[i] = 0;
    }

    receiver->lastUpdate = 0;
    receiver->currentPos = 0;
}

/**
 * @brief Funtion to call in interrupt handler to calculate the length of each channel
 * 
 * @param receiver device to handle interrupt for
 */
void FSIA10B_INT(FSIA10B *receiver, TIM_HandleTypeDef *htim){
    uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    // 1 tick = 1uS
    // Normal length from 1ms - 2ms, over 2ms is gap between last and first signal
    // (min of the inteval signal is 4ms)
    if(val >= FSIA10B_TIME_START_GREATER || receiver->currentPos == FSIA10B_NUMBER_CHANNELS){
        receiver->lastUpdate = HAL_GetTick();
        receiver->currentPos = 0;
    }else{
        receiver->channels[receiver->currentPos++] = val;
    }

    // Reset
    __HAL_TIM_SET_COUNTER(htim, 0);
}

/**
 * @brief Function to check that the receiver is receiving a signal
 * 
 * @param receiver the receiver to check
 * @return uint8_t boolean value 0 if not received within TIMEOUT time, 1 if receiving
 */
uint8_t FSIA10B_isReceivingSignal(FSIA10B *receiver){
    return (HAL_GetTick() - receiver->lastUpdate) < FSIA10B_RECEIVER_TIMEOUT;
}