#ifndef MOTOR_H
#define MOTOR_H
#include "config.h"
#include "odometry.h"
#include <msp430.h>
#include <stdint.h>
// defines the behaviour and configuration of the dc motors using the dollatek
// dc controller with the TB6612FNG chip, this driver regulates the velocity of
// the motor changin the duty cycle of the PWM that we send

// PWMA P1.2 / TA0.1
// PWMB P1.3 / TA0.2
// AIN1 P1.4
// AIN2 P1.5
// BIN1 P2.0
// BIN2 P2.6
// STBY P8.2

//=====MOTOR COMMANDS=====

static inline void mt_left_forward() {
    P1OUT |= BIT4;
    P1OUT &= ~BIT5;
    enc_left_dir = 1;
}
static inline void mt_right_forward() {
    P2OUT |= BIT0;
    P2OUT &= ~BIT6;
    enc_right_dir = 1;
}
static inline void mt_forward() {
    mt_left_forward();
    mt_right_forward();
}

//------------------

static inline void mt_left_stop() {
    P1OUT |= BIT5;
    P1OUT |= BIT4;
}
static inline void mt_right_stop() {
    P2OUT |= BIT0;
    P2OUT |= BIT6;
}
static inline void mt_stop() {
    mt_left_stop();
    mt_right_stop();
}
//-----------
static inline void mt_left_backward() {
    P1OUT |= BIT5;
    P1OUT &= ~BIT4;
    enc_left_dir = -1;
}
static inline void mt_right_backward() {
    P2OUT |= BIT6;
    P2OUT &= ~BIT0;
    enc_right_dir = -1;
}
static inline void mt_backward() {
    mt_left_backward();
    mt_right_backward();
}
static inline void mt_change_left_speed(uint8_t new_speed){
    if (new_speed > 50){
        new_speed = 50;
    }
    TA0CCR1 = new_speed;
    
}
static inline void mt_change_right_speed(uint8_t new_speed){
    if (new_speed > 50){
        new_speed = 50;
    }
    TA0CCR2 = new_speed;
    
}
// changes the motor speed, it goes from 0 to 50
static inline void mt_change_speed(uint8_t new_speed) {
    mt_change_left_speed(new_speed);
    mt_change_right_speed(new_speed);
}

// configures the timer and the pins the motor needs to use
void config_mt(void);
// changes the behaviour of the motor depending of the receiving char
void update_mt_with_bt(char message[BT_RX_BUF_SIZE]);
#endif
