#include "Drone.h"

static void Drone_armedLoop(Drone* drone);
static void Drone_armingLoop(Drone* drone);
static void Drone_disarmedLoop(Drone* drone);
static void Drone_disarm(Drone* drone);
static uint8_t Drone_canFly(Drone* drone);

/**
 * @brief Function to initialise the drone
 * 
 * @param drone The drone struct to initialise
 * @param motors The motors of the drone
 * @param battery The battery of the drone
 * @param mpu The MPU6050 of the drone
 * @param receiver The receiver of the drone
 */
void Drone_initialise(Drone* drone, ESC_4Channels* motors, Battery* battery, MPU6050* mpu, FSIA10B* receiver){
    drone->state = DISARMED;
    drone->motors = motors;
    drone->battery = battery;
    drone->mpu = mpu;
    drone->receiver = receiver;
}

void Drone_run(Drone* drone){
    switch(drone->state){
        case ARMED:
            Drone_armedLoop(drone);
            break;
        case DISARMED:
            Drone_disarmedLoop(drone);
            break;
        case ARMING:
            Drone_armingLoop(drone);
            break;
        default:
            // Shouldn't get to this if we do shutdown drone
            Drone_disarm(drone);
    }
}

/**
 * @brief Function to start the switching of the start from DISARMED to ARMED
 * 
 * @param drone The drone to start arming
 */
static void Drone_Arm(Drone* drone){
    drone->state = ARMING;
}

/**
 * @brief Function to switch the state of the drone from ARMED to DISARMED
 * 
 * @param drone The drone to disarm
 */
static void Drone_disarm(Drone* drone){
    drone->state = DISARMED;
    ESC_disarm(drone->motors);
}

/**
 * @brief Function to call in the loop when the drone is armed
 * 
 * @param drone The drone that is disarmed
 */
static void Drone_disarmedLoop(Drone* drone){

    // If drone can fly and the throttle is off arm the drone
    if(Drone_canFly(drone) && FSIA10B_throttleIsOff(drone->receiver)){
        Drone_Arm(drone);
    }

}

/**
 * @brief Function to call in the loop when the drone is arming (non-blocking)
 * 
 * @param drone The drone that is arming
 */
static void Drone_armingLoop(Drone* drone){
    if(ESC_armNonBlocking(drone->motors)){
        drone->state = ARMED;
    }
}

/**
 * @brief Function that checks whether all components are working allowing it to fly
 * 
 * @param drone The drone which needs to be checks
 * @return uint8_t boolean value 1 (true) if can fly (0) false if cannot
 */
static uint8_t Drone_canFly(Drone* drone){
    return  FSIA10B_isArmed(drone->receiver)            // The switch to arm the drone is enable of the controller
              &&
            Battery_hasCharge(drone->battery)           // The controller is over the minimum voltage that is safe
              &&
            FSIA10B_isReceivingSignal(drone->receiver)  // The Receiver is receiving up to date data
              &&
            MPU6050_isReadingData(drone->mpu);          // The MPU6050 is reading up to date data
}

/**
 * @brief Function to call in the loop when the drone is armed
 * 
 * @param drone The drone that is armeds
 */
static void Drone_armedLoop(Drone* drone){
    // Check all components functioning
    if(!Drone_canFly(drone)){
        Drone_disarm(drone);
        return;
    }

    // Write throttle channel to all channels
    if(drone->receiver->channels[2] < 1000){
        ESC_writeAll(drone->motors, 1000);
    }else if(drone->receiver->channels[2] > 2000){
        ESC_writeAll(drone->motors, 2000);
    }else{
        ESC_writeAll(drone->motors, drone->receiver->channels[2]);
    }
}