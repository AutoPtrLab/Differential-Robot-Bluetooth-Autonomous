//  The encoders used in the project are relative , we are using a U-Shape light
// sensor
//
// Left encoder P2.7
// Right encoder P1.6
//
// the IMU used is the MPU650 with the acelerometer and gyroscope
//
// Clock signal P3.1
// Data channel P3.0
// Interrupt P2.2

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "I2C_helper.h"
#include "math_helper.h"
#include "robot_info.h"

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// Q16.16 meters that the robot drives per pulse
#define Q16_PER_PULSE (q16_t)((WHEEL_DIAMETER_MM * Q16_PI) / (1000UL * (ENCODER_GAPS)))
// distance between the two wheels
#define WHEEL_WHEEL_DIST_Q16 (q16_t)(INT_TO_Q16(WHEEL_WHEEL_DIST_MM) / 1000)
// (1/2PI) * 2^32 , this is used to pass from q16 rad to bam angles
#define INV_2PI_X2_32 683565276

// ========GLOBAL VARIABLES=======

// this vartiable is used in the interrupt rutine
// in the task timer so is defined there
extern volatile bool _is_update_pos_ready;

// variablea that holds the count of the encoders, it decreases when its going
// backwards
extern volatile int16_t _enc_left_cont;
extern volatile int16_t _enc_right_cont;

// it represents the direction of the motor
extern volatile int8_t enc_left_dir;
extern volatile int8_t enc_right_dir;

// encoders count getters , they clean the counter everytime
static inline int16_t enc_get_left(void) { return _enc_left_cont; }
static inline int16_t enc_get_right(void) { return _enc_right_cont; }
// checks if the update flag is high and cleans the flag
static inline bool is_pos_update_ready_n_clean(void) {
    bool aux = _is_update_pos_ready;
    _is_update_pos_ready = false;
    return aux;
}

// returns the angular velocity in the z axis in Q16 degrees p
static inline q16_t get_gyro_z(void) {
    uint8_t arr[2] = {0, 0};
    I2C_read_mult_data(GYRO_ZOUT_H, arr, 2);

    // cast to int16 so the cast to q16_t does not eliminate the sign
    int16_t value = (int16_t)(((uint16_t)arr[0] << 8) |
                              (uint16_t)arr[1]); // conversion between two uint8 to the int16 value
                                                 // , we also divide by the
    // since the scaling to get deegres pero second is to divide by 131 , and we want q16 rad/s we
    // multiply Pi/180 and then * 65536 , this gives as a
    //  constant of aroung 8.73 , multiplying by 2^16 * 8.73 and then dividing by 2^16 get us the
    //  Q16 rad/s
    return (q16_t)(((int32_t)value * 572224L) >> 16);
}

// configuration of the encoder ports
void config_odometry(void);
// updates the position belief of the robot, it returns the pointer to the current belief
void update_odometry(Pose2D *pose, int16_t enc_left, int16_t enc_right);

// sets the gyroscope offset, receives the number of samples its going to take(the waiting for the
// nex measure is 0.2s)
int32_t set_gyro_offset(uint8_t samples);

#endif
