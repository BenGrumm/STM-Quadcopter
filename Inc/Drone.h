#ifndef DRONE_STATE_MACHING_H
#define DRONE_STATE_MACHINE_H

#include "BatteryMeasure4SLiPo.h"
#include "ESCController.h"
#include "FS-IA10B_driver.h"
#include "MPU6050.h"
#include "pid.h"
#include <stdlib.h>

#define PID_SAMPLE_FREQ   200.0
#define PID_SAMPLE_TIME_MILLIS (1000.0 / PID_SAMPLE_FREQ)

#define RATE_PITCH_kP   0.19 // 0.375 original (P only)
#define RATE_PITCH_kI   0.02
#define RATE_PITCH_kD   0.0

#define RATE_ROLL_kP    0.19 // 0.375 original (P only)
#define RATE_ROLL_kI    0.02
#define RATE_ROLL_kD    0.0

#define RATE_YAW_kP     0
#define RATE_YAW_kI     0
#define RATE_YAW_kD     0

#define RATE_PID_MIN_OUTPUT -400
#define RATE_PID_MAX_OUTPUT 400

#define RATE_SPEED_DPS                200.0F
#define RATE_MAX_ANGLE_PLUS_MINUS     32.8F

typedef enum {
    DISARMED = 0,
    ARMING,
    ARMED
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

  double rate_pitch_input, rate_pitch_output, rate_pitch_setpoint;
  double rate_roll_input, rate_roll_output, rate_roll_setpoint;
  double rate_yaw_input, rate_yaw_output, rate_yaw_setpoint;

  PID_TypeDef rate_pitch;
  PID_TypeDef rate_roll;
  PID_TypeDef rate_yaw;

  uint32_t lastLoop;
}Drone;

void Drone_initialise(Drone* drone, ESC_4Channels* motors, Battery* battery, MPU6050* mpu, FSIA10B* receiver);
void Drone_run(Drone* drone);
double Drone_clampValue(double num, double min, double max);

#endif