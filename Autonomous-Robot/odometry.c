#include "odometry.h"
#include "config.h"

#include "math_helper.h"
#include "msp430f5529.h"
#include "robot_info.h"
#include "trigonometry_helper.h"

#include <stdint.h>
//
// Left encoder 2.7
// Right encoder 1.6
//
//

// encoders

volatile int16_t _enc_left_cont = 0;
volatile int16_t _enc_right_cont = 0;

volatile int8_t enc_left_dir = 0;
volatile int8_t enc_right_dir = 0;
// gyroscope

q16_t _gyro_z_offset = 0;

q16_t debug_gyro = 0;

void config_odometry(void) {
    P1DIR &= ~BIT6;
    P2DIR &= ~BIT7;
    P2REN |= BIT7;
    P2OUT |= BIT7;  // Pull up res
    P2SEL &= ~BIT7; //

    P1REN |= BIT6;  //
    P1OUT |= BIT6;  // Pull up res
    P1SEL &= ~BIT6; // GPIO
    // clean int flags
    P1IFG &= ~BIT6;
    P2IFG &= ~BIT7;

    P1IES |= BIT6;
    P2IES |= BIT7;

    // activate the interrupts
    P2IE |= BIT7;
    P1IE |= BIT6;
    // the configutration for the debug button

    config_I2C(); // configures the connection with the IMU

    // // we enable the MPU
    I2C_write_data(PWR_MGMT_1, 0); // we set the sleep to zero

    _gyro_z_offset = set_gyro_offset(10);
}

// private function to filtrate the IMU signal(is giving peaks) using a median filter
q16_t get_filtered_gyro(void) {
    static q16_t gyro_data[4] = {0, 0, 0, 0}; // array of past values
    static uint8_t gyro_id = 0;               // index of the gyro data
    // we get the raw data
    q16_t raw = get_gyro_z() - _gyro_z_offset;
    gyro_data[gyro_id] = raw;
    gyro_id = (gyro_id + 1) & 3; // & 3 == % 4

    // temp array to make the modifications and not touching the original array
    q16_t temp[4];
    for (int i = 0; i < 4; i++) {
        temp[i] = gyro_data[i];
    }
    // bubble sort
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                q16_t aux = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = aux;
            }
        }
    }
    // median of the array
    q16_t filtered_val = (temp[1] + temp[2]) >> 2;
    // dead zone to prevent small noise
    if (filtered_val < 600 && filtered_val > -600) {
        filtered_val = 0;
    }

    return filtered_val;
}
void update_odometry(Pose2D *pose, int16_t enc_left, int16_t enc_right) {

    // calculate the delta time
    static uint16_t prev_time = 0; // variable that holds the previous instance this func was called
    __bic_SR_register(GIE);
    uint16_t current_time = TB0R;
    __bis_SR_register(GIE);
    uint16_t dt = current_time - prev_time; // delta time is in 1/32728 seconds
    prev_time = current_time;

    q16_t ang_vel_q16 = get_filtered_gyro(); // filter the IMU signal

    debug_gyro = ang_vel_q16;
    // DISTANCE CALCULATION

    // dx = enc_cont * mm_per_pulse
    // D = (dl+dr)/2
    q16_t dist_left = enc_left * Q16_PER_PULSE;   // distance the left wheel travels
    q16_t dist_right = enc_right * Q16_PER_PULSE; // distance the right wheel travels
    q16_t dist = (dist_left + dist_right) /
                 2; // Distance in Q16 meters taking in reference to the rotation center

    // ANGULAR CALCULATION

    // rad = (dl - dr)/ (2*wheel_rot_center == wheel_wheel_dist)

    q16_t delta_theta_q16 = Q16_div(dist_right - dist_left, WHEEL_WHEEL_DIST_Q16);

    // since we want to pass from the Q16 rad to the bam representation (which is a normalization of
    // a circle between the values of a uint16_t ) we should first mulitply by 65536 and then divide
    // by 2PI (since is in radians) but since we are using q16 rads (65536 * rad) we only need to
    // divide by 2PI
    //
    // the 683565276 is the simplification of (1/2PI * 2^32) since we dont want to lose precision in
    // the division
    ang_unit delta_theta = (ang_unit)(((int64_t)delta_theta_q16 * INV_2PI_X2_32) >> 32);

    // calculation of the angle
    q16_t gyro_delta_q16 = (q16_t)(((int64_t)ang_vel_q16 * (int64_t)dt) / 32768LL);
    gyro_delta_q16 = (q16_t)((int64_t)((int64_t)ang_vel_q16 * POSITION_UPDATE_MS) / 1000);
    ang_unit gyro_delta = (ang_unit)(((int64_t)gyro_delta_q16 * INV_2PI_X2_32) >> 32);

    // cast to int  so we can have negative numbers back and ca
    int16_t delta_gyr = (int16_t)gyro_delta;
    int16_t delta_enc = (int16_t)delta_theta;
    // complementary filter normalize for dividing by 1024
    int32_t mixed_delta = ((int32_t)delta_gyr * 973) + ((int32_t)delta_enc * 51);

    int16_t fused_delta = (int16_t)(mixed_delta >> 10);
    // mid point aproximation of the angle
    ang_unit mid_angle = (ang_unit)(pose->ang + (fused_delta / 2));
    // we calculate the new position before assigning the new angle
    pose->pos.x += Q16_mult(dist, Q16_cos(mid_angle));
    pose->pos.y += Q16_mult(dist, Q16_sin(mid_angle));
    // we sum the angle
    pose->ang = pose->ang + (ang_unit)fused_delta;
}
q16_t set_gyro_offset(uint8_t samples) {
    int64_t total = 0;
    for (int i = 0; i < samples; i++) {
        total += get_gyro_z();
        __delay_cycles(60000); // waits 0.2 sec
    }
    return total / samples;
}

//===========\
// ---ISR----
// ==========

#pragma vector = PORT1_VECTOR
__interrupt void port1_ISR(void) {

    if (P1IFG & BIT6) {
        _enc_right_cont += (int16_t)enc_right_dir;
        P1IES ^= BIT6;  // we toggle the flank selection so it detecs the rise and
                        // fall of the pulse
        P1IFG &= ~BIT6; // clean the int flag
    }
    if (P1IFG & BIT7) { // debug stop
        P1IFG &= ~BIT6;
    }
}

#pragma vector = PORT2_VECTOR
__interrupt void port2_ISR(void) {
    if (P2IFG & BIT7) {

        _enc_left_cont += (int16_t)enc_left_dir;
        P2IES ^= BIT7;  // we toggle the flank selection so it detecs the rise and
                        // fall of the pulse
        P2IFG &= ~BIT7; // clean the int flag
    }
}

// #pragma vector = USCI_B0_VECTOR
// __interrupt void I2C_ISR() {
//     if (UCB0IFG & UCRXIFG) {
//     }
// }
