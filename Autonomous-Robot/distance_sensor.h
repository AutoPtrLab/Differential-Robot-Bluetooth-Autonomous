// The sensor used in this code is the HC-SR04 , to get the distance of the
// object we have to divide the time of the pulse in the echo pin(in
// microsecons) by 58 so Dist = T / 58
//
// the time is provided by the echo pin , it sends a pulse (150 micro sec - 25
// ms, 38 ms when there is no obstacule) we activate the sensor sending a 10
// microsec pulse to the trigger pin
//
// To measure the time the timerA2 is used.
// Since the autonomous car has
// 3 sensors, two on the sides and one on the front the used pins are the
// following:
//
// Front Sensor:
//      Trig = P4.2
//      Echo = P2.5 / TA2.2
// Left Sensor:
//      Trig = P3.2
//      Echo = P2.4 / TA2.1
// Right Sensor:
//      Trig = P8.1
//      Echo = P2.3 / TA2.0

#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H
// global variables
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
extern volatile uint16_t _ds_front_time;
extern volatile uint16_t _ds_left_time;
extern volatile uint16_t _ds_right_time;

extern volatile bool _ready_front;
extern volatile bool _ready_left;
extern volatile bool _ready_right;

// get the front time and turns off the is_ready flag
static inline uint16_t ds_get_front_time() {
  _ready_front = false;
  return _ds_front_time;
}
static inline uint16_t ds_get_left_time() {
  _ready_left = false;
  return _ds_left_time;
}
static inline uint16_t ds_get_right_time() {
  _ready_right = false;
  return _ds_right_time;
}

static inline uint16_t ds_right_is_ready() { return _ready_right; }
static inline uint16_t ds_left_is_ready() { return _ready_left; }
static inline uint16_t ds_front_is_ready() { return _ready_front; }

// configures the ports that of the distance sensor
void config_ds();

// triggers the front sensor
static inline void ds_trig_front() {
  P4OUT |= BIT2;
  __delay_cycles(10);
  P4OUT &= ~BIT2;
}
// triggers the left sensor
static inline void ds_trig_left() {
  P3OUT |= BIT2;
  __delay_cycles(10);
  P3OUT &= ~BIT2;
}
// triggers the right sensor
static inline void ds_trig_right() {
  P8OUT |= BIT1;
  __delay_cycles(10);
  P8OUT &= ~BIT1;
}

#endif
