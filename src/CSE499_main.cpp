/**
 * @file main.cpp
 * @date 2022-04-07
 * @author Andrew W, Imani M-G, Daniel M
 * @brief CSE 499 self balancing robot project implementation
 * last updated 2022-04-21
 */

#include "mbed.h"
#include "CSE499_main.h"
#include "MMA8452.h"

/* instantiate objects and variables */

MMA8452 accel(PB_9, PB_8, 100000);

DigitalIn tiltHF(PD_14);
DigitalIn tiltSF(PD_15);
DigitalIn tiltSB(PD_12);
DigitalIn tiltHB(PD_11);

DigitalOut tiltF(PF_12);
DigitalOut tiltB(PD_9);

DigitalOut m1(PF_13);
DigitalOut ml1(PE_9);
DigitalOut ml2(PE_11);
DigitalOut mr1(PF_14);
DigitalOut mr2(PE_13);

uint8_t tilt;
int prevRoll;
long dt;

/* implementation */

int main() {
    // setup variables to hold important data
    double accel_data[3] = { 0 };
    double roll = 0.0;

    setup();
    // tiltF = 1;
    // tiltB = 1;
    while (true) {
        // tilt = checkTilt();
        ml1 = !ml1;
        ml2 = !ml2;
        mr1 = !mr1;
        mr2 = !mr2;
        // check if I2C ready
        if ( !accel.isXYZReady() ) {
            ThisThread::sleep_for(2ms);
        }
        controller();
        // get accelerometer output
        //accel.readXYZGravity(&accel_data[0], &accel_data[1], &accel_data[2]);
        // get tilt angle
        //roll = calcRoll(accel_data[0], accel_data[2]);
        //printf("Grav: %3.2f\t%3.2f\t%3.2f\t%3.2f\n\r", accel_data[0], accel_data[1], accel_data[2], roll);
        // printf("%d\n\r", tilt);
        //ThisThread::sleep_for(1s);
    }
}


void setup() {
    accel.setBitDepth(MMA8452::BIT_DEPTH_12);
    accel.setDynamicRange(MMA8452::DYNAMIC_RANGE_2G);
    accel.setDataRate(MMA8452::RATE_100);
    m1  = 1;
    ml1 = 0;
    ml2 = 0;
    mr1 = 0;
    mr2 = 0;
    tilt = BALANCED;
}


void controller() {
    // get accelerometer output
    accel.readXYZGravity(&accel_data[0], &accel_data[1], &accel_data[2]);
    // get tilt anglle
    roll = calcRoll(accel_data[0], accel_data[2]);
    error = abs(roll) - 180; // current - target
    errorSum += error;
    motorPower = Kp*error + Ki*errorSum*dt - Kd*(roll-prevRoll)/dt;
    prevRoll = roll;
    // set motors
}
    
    
void setMotors(boolean forwards) {
    if ( forwards ) {
        ml1 = 1;
        ml2 = 0;
        mr1 = 0;
        mr2 = 1;
    }
    else {
        ml1 = 0;
        ml2 = 1;
        mr1 = 1;
        mr2 = 0;
    }
}


double calcRoll(double x, double z) {
    return atan2(x,z) * RAD_TO_DEG;
}


tilt_pos_t checkTilt() {
    if ( tiltHF == 1 ) {
        return HARDFRONT;
    }
    if ( tiltHB ) {
        return HARDBACK;
    }
    if ( tiltSF == 1 ) {
        return SOFTFRONT;
    }
    if ( tiltSB ) {
        return SOFTBACK;
    }
    return BALANCED;
}

