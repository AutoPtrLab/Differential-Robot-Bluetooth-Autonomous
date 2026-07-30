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

#include <stdbool.h>
#include <stdint.h>
extern volatile uint16_t _ds_front_time;
extern volatile uint16_t _ds_left_time;
extern volatile uint16_t _ds_right_time;

extern volatile bool _ready_front;
extern volatile bool _ready_left;
extern volatile bool _ready_right;

// getters
static inline uint16_t ds_get_front_time() { return _ds_front_time; }
static inline uint16_t ds_get_left_time() { return _ds_left_time; }
static inline uint16_t ds_get_right_time() { return _ds_right_time; }

static inline uint16_t ds_right_is_ready() { return _ready_right; }
static inline uint16_t ds_left_is_ready() { return _ready_left; }
static inline uint16_t ds_front_is_ready() { return _ready_front; }

// configures the ports that of the distance sensor
void config_ds();

// triggers the left distance sensor (blocks the CPU)
static inline void ds_trig_left();
// triggers the front distance sensor (blocks the CPU)
static inline void ds_trig_front();
// triggers the right distance sensor (blocks the CPU)
static inline void ds_trig_right();

#endif
