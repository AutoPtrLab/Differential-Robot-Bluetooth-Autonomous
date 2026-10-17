#include "math_helper.h"
#include "odometry.h"
#include "control.h"

#include "config.h"
#include "motor.h"

#include <stdint.h>
// Proportional constant of the PID controller is 1/ 2^15 so we make a bitshift
#define TWO_PI_URAD 6283185   // 360 degrees in microrads
#define PI_URAD (6283185 / 2) // 180 degrees in microrads
#define ANG_TOLERANCE 60000   // in microradians

#define CONTROL_DT (CONTROL_UPDATE_MS * 1000)
static int32_t angular_diff(int32_t a, int32_t b) {
    int32_t diff = a - b;
    if (diff < 0) {
        diff = -diff;
    }
    return (diff > PI_URAD) ? (TWO_PI_URAD - diff) : diff;
}



void update_control(int32_t linear_vel,int32_t ang_vel,int16_t left_enc, int16_t right_enc){ 
    //distance is the error since we want that the distacne to be 0 
    q16_t left_vel = Q16_mult(left_enc ,Q16_div_nat(Q16_PER_PULSE , CONTROL_DT)) ;
    q16_t right_vel = Q16_mult(right_enc ,Q16_div_nat(Q16_PER_PULSE , CONTROL_DT)) ;
    q16_t desired_right_vel = linear_vel + Q16_mult(ang_vel ,Q16_div_nat( WHEEL_WHEEL_DIST_Q16 , 2));
    q16_t desired_left_vel = linear_vel - Q16_mult(ang_vel ,Q16_div_nat( WHEEL_WHEEL_DIST_Q16 , 2));
    q16_t left_err =desired_left_vel - left_vel;
    q16_t right_err = desired_right_vel - right_vel ;
    
    mt_change_left_speed(left_err * Kp  );
    mt_change_right_speed(right_err * Kp );
}

