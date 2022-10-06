#ifndef BATTERY_MEASURE_4S_LIPO_HEADER
#define BATTERY_MEASURE_4S_LIPO_HEADER

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>

/**
 * Equation 1: RAIN max formula
 * For error below 1/4 of LSB should use impedance of < 50 kOhm
 */

// 12 bit value from 0-4095 with 3.3v = 4095 & 0v = 0
#define ADC_TO_VOLTAGE_VAL 1240.9f

typedef struct {
    uint32_t resistor_one;
    uint32_t resistor_two;
    float voltage;
}Battery;

void BatteryADCIRQ(ADC_HandleTypeDef* hadc1, Battery* battery);

#endif