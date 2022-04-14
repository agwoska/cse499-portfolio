/**
 * @file main.cpp
 * @date 2022-04-07
 * @author Andrew W, Imani M-G, Daniel M
 * last updated 2022-04-14
 */

#include "mbed.h"
#include "CSE499_main.h"

// fix pins later
DigitalIn tiltHF(PD_14);
DigitalIn tiltSF(PD_15);
DigitalIn tiltSB(PD_12);
DigitalIn tiltHB(PD_11);

DigitalOut tiltF(PF_12);
DigitalOut tiltB(PD_9);

DigitalOut m1(PF_13);
DigitalOut m2(PE_9);
DigitalOut m3(PE_11);


int main() {
    tiltF = 1;
    tiltB = 1;
    m1 = 1;
    m2 = 1;
    m3 = 0;
    uint8_t tilt = BALANCED;
    while (true) {
        ThisThread::sleep_for(1s);
        tilt = checkTilt();
        printf("%d\n\r", tilt);
    }
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
