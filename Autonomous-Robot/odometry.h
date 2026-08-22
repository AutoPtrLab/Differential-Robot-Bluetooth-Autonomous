//
//
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
#include "trigonometry_helper.h"
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// private defines
// micrometers that the robot drives per pulse divided by two,
#define UM_PER_PULSE ((uint32_t)(WHEEL_DIAMETER_MM) * 3141592UL / (1000UL * (ENCODER_GAPS) * 2UL))
#define WHEEL_WHEEL_DIST_UM (uint32_t)(WHEEL_WHEEL_DIST_MM * 1000UL)

//===ROBOT POSITION STRUCT ===
typedef struct {
    int32_t x;   // micrometers
    int32_t y;   // micrometers
    int32_t ang; // the radians have been scaled 1000000
} Position;      // postition in micrometers

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
static inline int16_t enc_get_n_clean_left() {
    int16_t aux = _enc_left_cont;
    _enc_left_cont = 0;
    return aux;
}
static inline int16_t enc_get_n_clean_right() {
    int16_t aux = _enc_right_cont;
    _enc_right_cont = 0;
    return aux;
}
// checks if the update flag is high and cleans the flag
static inline bool is_pos_update_ready_n_clean() {
    bool aux = _is_update_pos_ready;
    _is_update_pos_ready = false;
    return aux;
}

// returns the angular velocity in the z axis in microRAD/S
static inline int32_t get_gyro_z() {
    uint8_t arr[2];
    I2C_read_mult_data(GYRO_ZOUT_H, arr, 2);

    return (((int32_t)((arr[0] << 8) | arr[1])) * URAD_PER_DEGREE) /
           131; // conversion between two uint8 to the int16 value , we also divide by the
                // sensitivity scale factor
}

// configuration of the encoder ports
void config_odometry();
// updates the position belief of the robot, it returns the pointer to the current belief
Position *update_odometry();

// sets the gyroscope offset, receives the number of samples its going to take(the waiting for the
// nex measure is 0.2s)
int32_t set_gyro_offset(uint8_t samples);

#endif
