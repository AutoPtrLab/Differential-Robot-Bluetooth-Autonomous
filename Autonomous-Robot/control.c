#include "control.h"
#include "math_helper.h"
#include "odometry.h"

#include "config.h"
#include "motor.h"
#include <stdint.h>
#include <stdlib.h>

// PI control constants (the integral constant has the time implicit inside it since dt is constant)
#define Kp INT_TO_Q16(100)
#define Ki INT_TO_Q16(10)
#define Kff INT_TO_Q16(30)

q16_t l_error_debug;
q16_t r_error_debug;
// value that represent how small an error has to be to be considered zero
#define TOLERANCE 5500
#define VEL_PER_PULSE (q16_t)(((int64_t)Q16_PER_PULSE * 1000LL) / CONTROL_UPDATE_MS)
#define WHEEL_DIST_HALF (WHEEL_WHEEL_DIST_Q16 / 2)
#define PWM_MAX 50
#define PWM_DEADBAND 16

void update_control(int16_t linear_vel, int16_t ang_vel, int16_t left_enc, int16_t right_enc) {

    q16_t left = linear_vel - ang_vel;
    q16_t right = linear_vel + ang_vel;

    if (left > 0) {
        mt_left_forward();

    } else if (left == 0) {
        mt_left_stop();
    } else {
        mt_left_backward();
        left = -left;
    }

    if (left < 20)
        left = 20;
    mt_change_left_speed((uint8_t)left);

    if (right > 0) {
        mt_right_forward();

    } else if (right == 0) {
        mt_right_stop();
    } else {
        mt_right_backward();
        right = -right;
    }

    if (right < 20)
        right = 20;
    mt_change_right_speed((uint8_t)right);
}

// void update_control(q16_t linear_vel, q16_t ang_vel, int16_t left_enc, int16_t right_enc) {
//
//     // definition of integral control variables
//     static q16_t left_integral = 0;
//     static q16_t right_integral = 0;

//     q16_t left_vel = left_enc * VEL_PER_PULSE;
//     q16_t right_vel = right_enc * VEL_PER_PULSE;

//     q16_t angular_component = Q16_mult(ang_vel, WHEEL_DIST_HALF);
//     q16_t desired_right_vel = linear_vel + Q16_mult(ang_vel, angular_component);
//     q16_t desired_left_vel = linear_vel - Q16_mult(ang_vel, angular_component);

//     q16_t left_err = desired_left_vel - left_vel;
//     q16_t right_err = desired_right_vel - right_vel;

//     // // if the error is nearly zero we set it
//     if (left_err > -TOLERANCE && left_err < TOLERANCE) {
//         left_err = 0;
//     }
//     if (right_err > -TOLERANCE && right_err < TOLERANCE) {
//         right_err = 0;
//     }
//     l_error_debug = left_err;
//     r_error_debug = right_err;
//     // since the control update occurs constanly at a fixed step we included dt in Ki,
//     // we are using an euler aproximation
//     left_integral += left_err;
//     right_integral += right_err;

//     q16_t limite_integral = INT_TO_Q16(5);

//     // no wind up
//     if (left_integral > limite_integral)
//         left_integral = limite_integral;
//     if (left_integral < -limite_integral)
//         left_integral = -limite_integral;

//     if (right_integral > limite_integral)
//         right_integral = limite_integral;
//     if (right_integral < -limite_integral)
//         right_integral = -limite_integral;

//     q16_t left_control_q16 = Q16_mult(left_err, Kp) + Q16_mult(left_integral, Ki);
//     q16_t right_control_q16 = Q16_mult(right_err, Kp) + Q16_mult(right_integral, Ki);

//     if (left_control_q16 > 0) {
//         mt_left_forward();
//     } else if (left_control_q16 == 0) {
//         mt_left_stop();
//     } else {
//         mt_left_backward();
//     }

//     if (right_control_q16 > 0) {
//         mt_right_forward();
//     } else if (right_control_q16 == 0) {
//         mt_right_stop();
//     } else {
//         mt_right_backward();
//     } // check for any overflow problems

//     int16_t left_speed = abs(Q16_TO_INT((left_control_q16)));
//     int16_t right_speed = abs(Q16_TO_INT(right_control_q16));

//     if (left_speed > 50)
//         left_speed = 50;
//     if (right_speed > 50)
//         right_speed = 50;

//     // dead band of the robot
//     if (left_speed > 0 && left_speed < 20) {
//         left_speed = 20;
//     }
//     if (right_speed > 0 && right_speed < 20) {
//         right_speed = 20;
//     }
//     mt_change_left_speed(map_deadband(left_speed));
//     mt_change_right_speed(map_deadband(right_speed));
// }
