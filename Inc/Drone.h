#ifndef DRONE_STATE_MACHING_H
#define DRONE_STATE_MACHINE_H

#include "BatteryMeasure4SLiPo.h"
#include "ESCController.h"
#include "FS-IA10B_driver.h"
#include "MPU6050.h"

typedef enum {
    POWER_ON = 0,
    ARMED,
    DISARMED
} DRONE_STATE;

typedef enum {
  NO_GYRO,      // Ensure Gyro is receiving data
  NO_RX,        // Not receiving signals from the controller
  LOW_VOLTAGE,  // Ensure battery voltage is not too low
  THROTTLE,     // Ensure Throttle = 0 at start
} ARM_PREVENTION_FLAGS;

typedef struct {
  DRONE_STATE state;
  ESC_4Channels* motors;
  Battery* battery;
  MPU6050* mpu;
  FSIA10B* receiver;
}Drone;

void Drone_initialise(Drone* drone, ESC_4Channels* motors, Battery* battery, MPU6050* mpu, FSIA10B* receiver);
void Drone_run(Drone* drone);

#endif