// The encoders used in the project are relative , we are using a U-Shape light
// sensor
//
// Left encoder 2.7
// Right encoder 1.6

#ifndef ENCODER_H
#define ENCODER_H

#include <msp430.h>
#include <stdint.h>
// global private variables
//
// variable that holds how many ints has the sensor send
extern volatile uint16_t _enc_left_cont;
extern volatile uint16_t _enc_right_cont;

static inline uint16_t enc_get_left() { return _enc_left_cont; }
static inline uint16_t enc_get_right() { return _enc_right_cont; }

// configuration of the encoder ports
void config_enc();

#endif
