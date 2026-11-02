// The sensor used in this code is the HC-SR04 , to get the distance of the
// object we have to divide the time of the pulse in the echo pin(in
// microseconds) by 58 so Dist(cm) = T / 58
//
// the time is provided by the echo pin , it sends a pulse (150 micro sec - 25
// microsec pulse to the trigger pin
//
// To measure the time the timerA2 is used.
// Since the autonomous car has
// 3 sensors, two on the sides and one on the front the used pins are the
// following:
//
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
#include "math_helper.h"
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

// helper struct to make more readable the distance sensors
typedef struct {
    q16_t front_dist;
    q16_t left_dist;
    q16_t right_dist;
} SensorDistances;

extern volatile uint16_t _ds_front_time;
extern volatile uint16_t _ds_left_time;
extern volatile uint16_t _ds_right_time;

// get the front time and turns off the is_ready flag
// since we want to have the distance in q16_t meters we would want to multiply by 2^16 and then
// divide by 5800, that would give us a constant of 11.3, so to make the fixed point operation we
// will multiply by 45 and then divide by 4 (11.25) this funcitons are made to be polled and retunr
// the most recent values

// return the front distance,we need to check if,should be used with the GIE off
static inline q16_t ds_get_front_dist(void) { return (q16_t)(_ds_front_time * 45) >> 4; }
// return the left distance,we need to check if,should be used with the GIE off
static inline q16_t ds_get_left_dist(void) { return (q16_t)(_ds_left_time * 45) >> 4; }
// return the right distance,we need to check if,should be used with the GIE off
static inline q16_t ds_get_right_dist(void) { return (q16_t)(_ds_right_time * 45) >> 4; }
// return all the distances in the array
static inline SensorDistances ds_get_dist(void) {
    SensorDistances aux;
    __bic_SR_register(GIE);
    aux.front_dist = ds_get_front_dist();
    aux.left_dist = ds_get_left_dist();
    aux.right_dist = ds_get_right_dist();
    __bis_SR_register(GIE);

    return aux;
}
// configures the ports that of the distance sensor
void config_ds(void);

// triggers the front sensor
static inline void ds_trig_front(void) {
    P4OUT |= BIT2;
    __delay_cycles(10);
    P4OUT &= ~BIT2;
}
// triggers the left sensor
static inline void ds_trig_left(void) {
    P3OUT |= BIT2;
    __delay_cycles(10);
    P3OUT &= ~BIT2;
}
// triggers the right sensor
static inline void ds_trig_right(void) {
    P8OUT |= BIT1;
    __delay_cycles(10);
    P8OUT &= ~BIT1;
}

#endif
