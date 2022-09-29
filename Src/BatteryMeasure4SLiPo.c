#include "BatteryMeasure4SLiPo.h"

/**
 * @brief Function to call in IRQ handler for ADC
 * 
 * @param hadc1 handler of ADC being used
 * @param battery battery struct containing info on resistors
 */
void BatteryADCIRQ(ADC_HandleTypeDef* hadc1, Battery* battery){
    uint16_t tempVal = HAL_ADC_GetValue(hadc1);
    float measuredVoltage = tempVal / ADC_TO_VOLTAGE_VAL;
    // To measure high voltage we use voltage divider so use that formula
    battery->voltage = floorf((measuredVoltage * (battery->resistor_one + battery->resistor_two) * 100.0) / battery->resistor_two) / 100.0;
}