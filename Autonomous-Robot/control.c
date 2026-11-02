#include "control.h"
#include "math_helper.h"
#include "odometry.h"

#include "config.h"
#include "motor.h"
#include <stdint.h>
#include <stdlib.h>

#define CONTROL_DT (CONTROL_UPDATE_MS * 1000)

// PI control constants (the integral constant has the time implicit inside it since dt is constant)
#define Kp 327680
#define Ki 32768

// value that represent how small an error has to be to be considered zero
#define TOLERANCE 2500

void update_control(q16_t linear_vel, q16_t ang_vel, int16_t left_enc, int16_t right_enc) {
    // definition of integral control variables
    static q16_t left_integral = 0;
    static q16_t right_integral = 0;
    // distance is the error since we nt that the distacne to be 0
    q16_t left_vel = Q16_mult(left_enc, Q16_div_nat(Q16_PER_PULSE, CONTROL_DT));
    q16_t right_vel = Q16_mult(right_enc, Q16_div_nat(Q16_PER_PULSE, CONTROL_DT));
    q16_t desired_right_vel = linear_vel + Q16_mult(ang_vel, Q16_div_nat(WHEEL_WHEEL_DIST_Q16, 2));
    q16_t desired_left_vel = linear_vel - Q16_mult(ang_vel, Q16_div_nat(WHEEL_WHEEL_DIST_Q16, 2));

    q16_t left_err = desired_left_vel - left_vel;
    q16_t right_err = desired_right_vel - right_vel;

    // // if the error is nearly zero we set it
    // if (left_err > -TOLERANCE && left_err < TOLERANCE) {
    //     left_err = 0;
    // }
    // if (right_err > -TOLERANCE && right_err < TOLERANCE) {
    //     right_err = 0;
    // }
    // since the control update occurs constanly at a fixed step we included dt in Ki,
    // we are using an euler aproximation
    left_integral += left_err;
    right_integral += right_err;

    q16_t limite_integral = INT_TO_Q16(5);

    // no wind up
    if (left_integral > limite_integral)
        left_integral = limite_integral;
    if (left_integral < -limite_integral)
        left_integral = -limite_integral;

    if (right_integral > limite_integral)
        right_integral = limite_integral;
    if (right_integral < -limite_integral)
        right_integral = -limite_integral;

    q16_t left_control_q16 = Q16_mult(left_err, Kp) + Q16_mult(left_integral, Ki);
    q16_t right_control_q16 = Q16_mult(right_err, Kp) + Q16_mult(right_integral, Ki);

    if (left_control_q16 > 0) {
        mt_left_forward();
    } else if (left_control_q16 == 0) {
        mt_left_stop();
    } else {
        mt_left_backward();
    }

    if (right_control_q16 > 0) {
        mt_right_forward();
    } else if (right_control_q16 == 0) {
        mt_right_stop();
    } else {
        mt_right_backward();
    } // check for any overflow problems

    int16_t left_speed = abs(Q16_TO_INT((left_control_q16)));
    int16_t right_speed = abs(Q16_TO_INT(right_control_q16));

    if (left_speed > 50)
        left_speed = 50;
    if (right_speed > 50)
        right_speed = 50;

    // dead band of the robot
    if (left_speed > 0 && left_speed < 20) {
        left_speed = 20;
    }
    if (right_speed > 0 && right_speed < 20) {
        right_speed = 20;
    }
    mt_change_left_speed((uint8_t)left_speed);
    mt_change_right_speed((uint8_t)right_speed);
}
