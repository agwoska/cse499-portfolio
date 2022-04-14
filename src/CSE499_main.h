/**
 * @file main.cpp
 * @date 2022-04-14
 * @author Andrew W, Imani M-G, Daniel M
 * last updated 2022-04-14
 */


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
 * checks the tilt angle
 * @return tilt level
 */
 tilt_pos_t checkTilt();
