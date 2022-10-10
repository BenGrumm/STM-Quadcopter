#include "Drone.h"

static void Drone_powerOnLoop(Drone* drone);
static void Drone_armedLoop(Drone* drone);
static void Drone_disarmedLoop(Drone* drone);
static void Drone_shutdown(Drone* drone);

void Drone_initialise(Drone* drone){

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

}

static void Drone_switchDisarmed(Drone* drone){

}

static void Drone_powerOnLoop(Drone* drone){

} 

static void Drone_armedLoop(Drone* drone){

}

static void Drone_disarmedLoop(Drone* drone){

}

static void Drone_shutdown(Drone* drone){

}