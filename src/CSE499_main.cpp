/**
 * @file main.cpp
 * @date 2022-04-07
 * @author Andrew W, Imani M-G, Daniel M
 * @brief CSE 499 self balancing robot project implementation
 * last updated 2022-05-10
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

Timer t;

// safety elements
Mutex mut;
Watchdog &wd = Watchdog::get_instance();

uint8_t tilt;
double accel_data[3] = { 0 };
double roll = 0.0;
int prevRoll;
long dt;
int errorSum;

/* implementation */

int main() {
    setup();
    while (true) {
        // tilt = checkTilt();
        // check if I2C ready
        if ( !accel.isXYZReady() ) {
            ThisThread::sleep_for(2ms);
        }
        controller();
        ThisThread::sleep_for(10ms);
        wd.kick();
    }
}


void setup() {
    accel.setBitDepth(MMA8452::BIT_DEPTH_12);
    accel.setDynamicRange(MMA8452::DYNAMIC_RANGE_2G);
    accel.setDataRate(MMA8452::RATE_100);
    wd.start(TIMEOUT_MS);
    m1  = 1;
    ml1 = 0;
    ml2 = 0;
    mr1 = 0;
    mr2 = 0;
    prevRoll = 0;
    t.start();
    dt = t.read_ms();
    errorSum = 0;
    tilt = BALANCED;
}


chrono::milliseconds controller_helper(int roll) {
    int r = abs( roll );
    if ( 170 < r ) {
        return 200ms;
    }
    if ( 165 < r ) {
        return 300ms;
    }
    if( 160 < r ) {
        return 400ms;
    }
    return 500ms;
}


void controller() {
    // get accelerometer output
    mut.lock();
    accel.readXYZGravity(&accel_data[0], &accel_data[1], &accel_data[2]);
    mut.unlock();
    // get tilt anglle
    roll = calcRoll(accel_data[0], accel_data[2]);
    int error = abs(roll) - 180; // current - target
    errorSum += error;
    // dt = t.read_ms() - dt;
    int motorPower = Kp*error + Ki*errorSum*dt - Kd*(roll-prevRoll)/dt;
    prevRoll = roll;
    // set motors
    if ( abs(roll) > 178 ) {
        // do nothing, its balanced
    }
    else if ( motorPower < 0 ) { // forwards
        setMotors(1, 0);
        ThisThread::sleep_for( controller_helper(roll) );
        setMotors(1, 1);
    }
    else {  // backwards
        setMotors(0, 0);
        ThisThread::sleep_for( controller_helper(roll) );
        setMotors(0, 1);
    }
    printf("motor: %d\troll: %3.1f\n\r", motorPower, roll);
}
    
    
void setMotors(int forwards, int stop) {
    if ( stop ) {
        ml1 = 0;
        ml2 = 0;
        mr1 = 0;
        mr2 = 0;
    }
    else if ( forwards ) {
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
    printf("%d\t%d\t%d\t%d\n\r", ml1.read(), ml2.read(), mr1.read(), mr2.read());
}


double calcRoll(double x, double z) {
    return atan2(-x,z) * RAD_TO_DEG;
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