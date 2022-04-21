/**
 * @file main.cpp
 * @date 2022-04-14
 * @author Andrew W, Imani M-G, Daniel M
 * @brief CSE 499 self balancing robot project header
 * last updated 2022-04-21
 */

 #pragma once

 #ifndef CSE499_MAIN_H
 #define CSE499_MAIN_H

/** constants **/

#define PI          ( 3.14159 )
#define RAD_TO_DEG  ( 180.0 / (PI) )

/** structure **/

typedef enum {
    BALANCED  = 0,
    HARDFRONT = 1,
    SOFTFRONT = 2,
    SOFTBACK  = 3,
    HARDBACK  = 4,
} tilt_pos_t;


/** prototype function **/

/**
 * @brief sets up sensors
 */
void setup();

/**
 * @brief calculates the roll of the device
 * @param y the y-axis output from the accelerometer
 * @param z the z-axis output from the accelerometer
 * @return the calculated roll value 
 */
double calcRoll(double y, double z);

/**
 * checks the tilt angle
 * @return tilt level
 */
 tilt_pos_t checkTilt();

#endif // CSE499_MAIN_H
