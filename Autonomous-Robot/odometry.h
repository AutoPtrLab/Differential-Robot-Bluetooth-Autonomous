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
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// private defines
// micrometers that the robot drives per pulse divided by two,
#define UM_PER_PULSE                                                           \
  ((uint32_t)(WHEEL_DIAMETER_MM) * 3141592UL / (1000UL * (ENCODER_GAPS) * 2UL))
#define WHEEL_WHEEL_DIST_UM (uint32_t)(WHEEL_WHEEL_DIST_MM * 1000UL)
//===ROBOT POSITION STRUCT ===
typedef struct {
  int32_t x;   // micrometers
  int32_t y;   // micrometers
  int32_t rad; // the radians have been scaled 1000000
} Position;    // postition in micrometers

// ========GLOBAL VARIABLES=======

// position of the robot
extern Position _robot_pos; // private global pos

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
//=====GETTERS
// robot position getters

static inline uint16_t pos_get_x() { return _robot_pos.x; }
static inline uint16_t pos_get_y() { return _robot_pos.y; }
static inline bool pos_get_rad() { return _robot_pos.rad; }

// encoders count getters
static inline int16_t enc_get_left() { return _enc_left_cont; }
static inline int16_t enc_get_right() { return _enc_right_cont; }
// checks if the update flag is high
static inline bool is_pos_update_ready() { return _is_update_pos_ready; }

// sets the '0' position of the array with the high bit and the '1' with the low
// bit of the gyroscope data
static inline void get_gyro_z(uint8_t *arr) {
  read_mult_data(GYRO_ZOUT_H, arr, 2);
}

// configuration of the encoder ports
void config_odometry();
// updates the position belief of the robot
void update_odometry();

#endif
