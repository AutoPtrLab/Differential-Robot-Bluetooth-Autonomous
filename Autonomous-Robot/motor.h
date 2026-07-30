
#ifndef MOTOR_H
#define MOTOR_H
#include "config.h"
// defines the behaviour and configuration of the dc motors using the dollatek
// dc controller with the TB6612FNG chip, this driver regulates the velocity of
// the motor changin the duty cycle of the PWM that we send

// PWMA P1.2 / TA0.1
// PWMB P1.3 / TA0.2
// AIN1 P1.4
// AIN2 P1.5
// BIN1 P2.0
// BIN2 P2.6
// STBY P2.2]

// configures the timer and the pins the motor needs to use
void config_mt();
// changes the behaviour of the motor depending of the receiving char
void update_mt(char message[BT_BUF_SIZE]);
#endif
