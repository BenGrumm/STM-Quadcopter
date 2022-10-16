#include "Drone.h"

static void Drone_armedLoop(Drone* drone);
static void Drone_armingLoop(Drone* drone);
static void Drone_disarmedLoop(Drone* drone);
static void Drone_disarm(Drone* drone);
static uint8_t Drone_canFly(Drone* drone);
static void Drone_calculateRateSetpoints(Drone* drone);
static float Drone_calculateRateSetpoint(float angle, uint16_t pulseLength);
static float Drone_calculateYawRateSetpoint(uint16_t pulseLength, uint16_t throttle);

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
    drone->lastLoop = 0;

    PID(&drone->rate_pitch, &drone->rate_pitch_input, &drone->rate_pitch_output, &drone->rate_pitch_setpoint, 
        RATE_PITCH_kP, RATE_PITCH_kI, RATE_PITCH_kD, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&drone->rate_roll, &drone->rate_roll_input, &drone->rate_roll_output, &drone->rate_roll_setpoint, 
        RATE_ROLL_kP, RATE_ROLL_kI, RATE_ROLL_kD, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&drone->rate_yaw, &drone->rate_yaw_input, &drone->rate_yaw_output, &drone->rate_yaw_setpoint, 
        RATE_YAW_kP, RATE_YAW_kI, RATE_YAW_kD, _PID_P_ON_E, _PID_CD_DIRECT);

    PID_SetMode(&drone->rate_pitch, _PID_MODE_AUTOMATIC);
    PID_SetMode(&drone->rate_roll, _PID_MODE_AUTOMATIC);
    PID_SetMode(&drone->rate_yaw, _PID_MODE_AUTOMATIC);

    PID_SetSampleTime(&drone->rate_pitch, PID_SAMPLE_TIME_MILLIS);
    PID_SetSampleTime(&drone->rate_roll, PID_SAMPLE_TIME_MILLIS);
    PID_SetSampleTime(&drone->rate_yaw, PID_SAMPLE_TIME_MILLIS);

    PID_SetOutputLimits(&drone->rate_pitch, RATE_PID_MIN_OUTPUT, RATE_PID_MAX_OUTPUT);
    PID_SetOutputLimits(&drone->rate_roll, RATE_PID_MIN_OUTPUT, RATE_PID_MAX_OUTPUT);
    PID_SetOutputLimits(&drone->rate_yaw, RATE_PID_MIN_OUTPUT, RATE_PID_MAX_OUTPUT);
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
            // Battery_hasCharge(drone->battery)           // The controller is over the minimum voltage that is safe
            //   &&
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

    if(HAL_GetTick() - drone->lastLoop < PID_SAMPLE_TIME_MILLIS){
        return;
    }

    drone->lastLoop = HAL_GetTick();

    // Cacluate setpoint
    Drone_calculateRateSetpoints(drone);

    // Set the input values
    // For rate mode that is just the Gyro degrees per second value
    drone->rate_pitch_input = drone->mpu->gyro_degreesPerS[1] - 1.27;
    drone->rate_roll_input = drone->mpu->gyro_degreesPerS[0] + 2.93;
    drone->rate_yaw_input = drone->mpu->gyro_degreesPerS[2] + 1.45;

    // Run PID
    PID_Compute(&drone->rate_pitch);
    PID_Compute(&drone->rate_roll);
    PID_Compute(&drone->rate_yaw);

    // Cacluate motor mixing
    uint16_t frontLeft = drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE];
    uint16_t backRight = drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE];
    uint16_t frontRight = drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE];
    uint16_t backLeft = drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE];

    if(drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE] > 1050){
        frontLeft = frontLeft - drone->rate_yaw_output + drone->rate_pitch_output - drone->rate_roll_output;
        backRight = backRight - drone->rate_yaw_output - drone->rate_pitch_output + drone->rate_roll_output;
        frontRight = frontRight + drone->rate_yaw_output + drone->rate_pitch_output + drone->rate_roll_output;
        backLeft = backLeft + drone->rate_yaw_output - drone->rate_pitch_output - drone->rate_roll_output;
    }

    frontLeft = (uint16_t) Drone_clampValue(frontLeft, 1000, 2000);
    backRight = (uint16_t) Drone_clampValue(backRight, 1000, 2000);
    frontRight = (uint16_t) Drone_clampValue(frontRight, 1000, 2000);
    backLeft = (uint16_t) Drone_clampValue(backLeft, 1000, 2000);

    FusionQuaternion quat = FusionAhrsGetQuaternion(drone->mpu->ahrs);
    FusionEuler euler = FusionQuaternionToEuler(quat);

    // Edit the output for the battery voltage
    // printf("Yaw = %6.2f, Pitch = %6.2f, Roll = %6.2f, FL = %d, BR = %d, FR = %d, BL = %d\n", drone->rate_yaw_output, drone->rate_pitch_output, drone->rate_roll_output, frontLeft, backRight, frontRight, backLeft);
    printf("Yaw = %6.2f, Pitch = %6.2f, Roll = %6.2f, YI = %f, PI = %f, RI = %f, Yaw = %f, Pitch = %f, Roll = %f\n", 
            drone->rate_yaw_output, drone->rate_pitch_output, drone->rate_roll_output, 
            drone->rate_yaw_input, drone->rate_pitch_input, drone->rate_roll_input,
            euler.angle.yaw, euler.angle.pitch, euler.angle.roll);

    // Write output
    ESC_writeMotors(drone->motors, frontLeft, backRight, frontRight, backLeft);
}

static void Drone_calculateRateSetpoints(Drone* drone){
    FusionQuaternion quat = FusionAhrsGetQuaternion(drone->mpu->ahrs);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    // Map receiver roll and pitch from ±200°/s
    // Channels should be from 1000-2000 but may be slight error
    drone->rate_pitch_setpoint = (double) Drone_calculateRateSetpoint(
        euler.angle.pitch,
        (uint16_t)Drone_clampValue(drone->receiver->channels[FSIA10B_CHANNEL_PITCH], 1000, 2000)
    );

    drone->rate_roll_setpoint = (double) Drone_calculateRateSetpoint(
        euler.angle.roll,
        (uint16_t)Drone_clampValue(drone->receiver->channels[FSIA10B_CHANNEL_ROLL], 1000, 2000)
    );

    // Map receiver yaw from ±270°/s
    drone->rate_yaw_setpoint = Drone_calculateYawRateSetpoint(
        (uint16_t)Drone_clampValue(drone->receiver->channels[FSIA10B_CHANNEL_YAW], 1000, 2000),
        drone->receiver->channels[FSIA10B_CHANNEL_THROTTLE]
    );
}

static float Drone_calculateYawRateSetpoint(uint16_t pulseLength, uint16_t throttle){
    // Prevent yaw under certain throttle value
    if(throttle > 1050){
        return Drone_calculateRateSetpoint(0, pulseLength);
    }

    return 0;
}

static float Drone_calculateRateSetpoint(float angle, uint16_t pulseLength){
    float max_angle_adjust = angle * (500.0F / RATE_MAX_ANGLE_PLUS_MINUS); // To stick to a maximum angle
    float setPoint = 0;

    // Center aroun 0 (+- 500) with band in middle to allow for slight error
    if(pulseLength < 1490){
        setPoint = pulseLength - 1490;
    }else if(pulseLength > 1510){
        setPoint = pulseLength - 1510;
    }

    setPoint -= max_angle_adjust;
    setPoint /= (500.0F / RATE_SPEED_DPS); // Divide by 2.5 to move to ±200 (degrees per second, the max rate allowed)

    return (float)Drone_clampValue(setPoint, -200, 200);
}

/**
 * @brief Function to clamp a value to within a range
 * 
 * @param num 
 * @param min 
 * @param max 
 * @return double 
 */
double Drone_clampValue(double num, double min, double max){
    if(num < min){
        return min;
    }else if(num > max){
        return max;
    }

    return num;
}