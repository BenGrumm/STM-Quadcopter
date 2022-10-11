#include "Drone.h"

static void Drone_powerOnLoop(Drone* drone);
static void Drone_armedLoop(Drone* drone);
static void Drone_disarmedLoop(Drone* drone);
static void Drone_shutdown(Drone* drone);

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
            Drone_shutdown(drone);
    }
}

static void Drone_switchArmed(Drone* drone){
    ESC_arm(drone->motors);

    drone->state = ARMED;
}

static void Drone_switchDisarmed(Drone* drone){

}

static void Drone_powerOnLoop(Drone* drone){
    
    // Ensure min voltage met
    if(!Battery_hasCharge(drone->battery)){
        return;
    }

    if(!FSIA10B_isReceivingSignal(drone->receiver)){
        return;
    }

    if(!FSIA10B_isArmed(drone->receiver)){
        return;
    }

    if(!MPU6050_isReadingData(drone->mpu)){
        return;
    }

    if(!FSIA10B_throttleIsOff(drone->receiver)){
        return;
    }

    Drone_switchArmed(drone);
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

static void Drone_shutdown(Drone* drone){

}