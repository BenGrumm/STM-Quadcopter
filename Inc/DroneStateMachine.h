#ifndef DRONE_STATE_MACHING_H
#define DRONE_STATE_MACHINE_H

enum DRONE_STATE {
    DISCONNECTED = 0,
    ARMED
};

enum ARM_PREVENTION_FLAGS {
  NO_GYRO,      // Ensure Gyro is receiving data
  NO_RX,        // Not receiving signals from the controller
  LOW_VOLTAGE,  // Ensure battery voltage is not too low
  THROTTLE,     // Ensure Throttle = 0 at start
};

#endif