#include "Drone.h"

static void Drone_powerOnLoop(Drone* drone);
static void Drone_armedLoop(Drone* drone);
static void Drone_disarmedLoop(Drone* drone);
static void Drone_disarm(Drone* drone);
static uint8_t Drone_canFly(Drone* drone);

void Drone_initialise(Drone* drone, ESC_4Channels* motors, Battery* battery, MPU6050* mpu, FSIA10B* receiver){
    drone->state = POWER_ON;
    drone->motors = motors;
    drone->battery = battery;
    drone->mpu = mpu;
    drone->receiver = receiver;
}

void Drone_run(Drone* drone){
    switch(drone->state){
        case POWER_ON:
            Drone_powerOnLoop(drone);
            break;
        case ARMED:
            Drone_armedLoop(drone);
            break;
        case DISARMED:
            Drone_disarmedLoop(drone);
            break;
        default:
            // Shouldn't get to this if we do shutdown drone
            Drone_disarm(drone);
    }
}

static void Drone_Arm(Drone* drone){
    ESC_arm(drone->motors);

    drone->state = ARMED;
}

static void Drone_disarm(Drone* drone){

}

static void Drone_powerOnLoop(Drone* drone){

    // If drone can't fly or the throttle is on return
    if(!Drone_canFly(drone) || !FSIA10B_throttleIsOff(drone->receiver)){
        return;
    }

    // If all conditions met arm the drone
    Drone_Arm(drone);
}

static uint8_t Drone_canFly(Drone* drone){
    return  FSIA10B_isArmed(drone->receiver)            // The switch to arm the drone is enable of the controller
              &&
            Battery_hasCharge(drone->battery)           // The controller is over the minimum voltage that is safe
              &&
            FSIA10B_isReceivingSignal(drone->receiver)  // The Receiver is receiving up to date data
              &&
            MPU6050_isReadingData(drone->mpu);          // The MPU6050 is reading up to date data
}

static void Drone_armedLoop(Drone* drone){
    if(drone->receiver->channels[2] < 1000){
        ESC_writeAll(drone->motors, 1000);
    }else if(drone->receiver->channels[2] > 2000){
        ESC_writeAll(drone->motors, 2000);
    }else{
        ESC_writeAll(drone->motors, drone->receiver->channels[2]);
    }
}

static void Drone_disarmedLoop(Drone* drone){

}
