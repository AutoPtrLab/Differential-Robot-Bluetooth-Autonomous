#include "odometry.h"
#include "bluetooth.h"
#include "config.h"

#include "msp430f5529.h"

#include <stdint.h>
//
// Left encoder 2.7
// Right encoder 1.6
//
//
int32_t DEBUG_ANG = 0;

// encoders

volatile int16_t _enc_left_cont = 0;
volatile int16_t _enc_right_cont = 0;

volatile int8_t enc_left_dir = 0;
volatile int8_t enc_right_dir = 0;
// gyroscope

static int32_t _gyro_z_offset = 0;

//  private robot position
static Position _robot_pos = {0, 0, 0};

void config_odometry() {
    //-----ENCODER------
    // no need for an external resistance
    //
    // set the pins as entrance
    P1DIR &= ~BIT6;
    P2DIR &= ~BIT7;
    P2REN |= BIT7;
    P2OUT |= BIT7;  // Pull up res
    P2SEL &= ~BIT7; //

    P1REN |= BIT6;  //
    P1OUT |= BIT6;  // Pull up res
    P1SEL &= ~BIT6; // 1. Modo GPIO
    // clean int flags
    P1IFG &= ~BIT6;
    P2IFG &= ~BIT7;

    P1IES |= BIT6;
    P2IES |= BIT7;

    // activate the interrupts
    P2IE |= BIT7;
    P1IE |= BIT6;

    config_I2C(); // configures the connection with the IMU

    // we enable the MPU
    I2C_write_data(PWR_MGMT_1, 0); // we set the sleep to zero

    _gyro_z_offset = set_gyro_offset(10);
}

Position *update_odometry() {
    // calculate the delta time
    static uint16_t prev_time = 0; // variable that holds the previous instance this func was called
    uint16_t pres_time = TB0R;
    uint16_t dt = pres_time - prev_time; // delta time is in micro seconds
    prev_time = pres_time;

    dt = TB0R - dt;
    __bic_SR_register(GIE); // off the ints. , atomic block
    int16_t enc_left = enc_get_n_clean_left();
    int16_t enc_right = enc_get_n_clean_right();
    __bis_SR_register(GIE); // reactivation of the global ints

    int32_t ang_vel_microrad =
        get_gyro_z() - _gyro_z_offset; // we get the angular velocity but also substract the offset

    DEBUG_ANG = ang_vel_microrad;

    //  dx = enc_cont * mm_per_pulse
    //  D = (dl+dr)/2
    int32_t dist_left = enc_left * UM_PER_PULSE;   // distance the left wheel travels
    int32_t dist_right = enc_right * UM_PER_PULSE; // distance the right wheel travels
    int32_t dist = (dist_left + dist_right) /
                   2; // Distance in micrometers taking in reference to the rotation center
    // rad = (dl + dr)/ (2*wheel_rot_center == wheel_wheel_dist)
    int32_t delta_theta = (int32_t)((((int64_t)(dist_right - dist_left)) * 1000000) /
                                    WHEEL_WHEEL_DIST_UM); // rads * 1000000

    _robot_pos.x += dist * urad_cos(_robot_pos.ang + (delta_theta / 2)) / 10000;

    _robot_pos.y += dist * urad_sin(_robot_pos.ang + (delta_theta / 2)) / 10000;

    // // calculation of the angle to make it more readable
    int32_t gyro_delta = (int32_t)(((int64_t)ang_vel_microrad * dt) / 1000000LL);

    int64_t gyro_output = 9500LL * (int64_t)(gyro_delta + _robot_pos.ang);
    int64_t enc_output = 500LL * (int64_t)(delta_theta + _robot_pos.ang);

    _robot_pos.ang = (int32_t)((gyro_output + enc_output) / 10000LL); // sum of the two part and
                                                                      // normalitation of the alphas

    // we normalize the angle of the robot so it could only be 0-359,999...
    while (_robot_pos.ang >= TWO_PI_URAD) {
        _robot_pos.ang -= TWO_PI_URAD;
    }
    while (_robot_pos.ang < 0) {
        _robot_pos.ang += TWO_PI_URAD;
    }
    return &_robot_pos;
}

int32_t set_gyro_offset(uint8_t samples) {
    int64_t total = 0;
    for (int i = 0; i < samples; i++) {
        total += get_gyro_z();
        __delay_cycles(200000); // waits 0.5 sec
    }
    return total / samples;
}

#pragma vector = PORT1_VECTOR
__interrupt void port1_ISR() {

    if (P1IFG & BIT6) {
        P1OUT ^= BIT0;
        _enc_right_cont += (int16_t)enc_right_dir;
        P1IES ^= BIT6;  // we toggle the flank selection so it detecs the rise and
                        // fall of the pulse
        P1IFG &= ~BIT6; // clean the int flag
    }
}

#pragma vector = PORT2_VECTOR
__interrupt void port2_ISR() {
    P1OUT ^= BIT0;
    if (P2IFG & BIT7) {

        _enc_left_cont += (int16_t)enc_left_dir;
        P2IES ^= BIT7;  // we toggle the flank selection so it detecs the rise and
                        // fall of the pulse
        P2IFG &= ~BIT7; // clean the int flag
    }
}

#pragma vector = USCI_B0_VECTOR
__interrupt void I2C_ISR() {
    if (UCB0IFG & UCRXIFG) {
    }
}
