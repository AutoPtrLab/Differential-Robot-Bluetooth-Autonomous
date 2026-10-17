

#include "control.h"
#include "navigation.h"
#include "odometry.h"

#include "bluetooth.h"
#include "distance_sensor.h"

#include "robot_info.h"
#include "task_timer.h"

#include "motor.h"
#include "msp430f5529.h"
#include <stdint.h>

// foreground - background
// maybe add low power modes to optimize

void config_ports(void);
void init_variables(void);
//------------------------------------------------
// global variables that live the entire program
// ------------------------------------------------
static RobotInfo robot_info;
static VelData desired_vel;
static Pose2D desired_pos;

void init_variables(void) {
    robot_info = default_robot_info();
    desired_vel = (VelData){0, 0};
    desired_pos = (Pose2D){(Vec2){0, 0}, 0};
}
int main(void) {

    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    P1DIR |= BIT0;
    P4DIR |= BIT7;
    config_ports();

    // Variables that live the entire program
    init_variables();
    // activate the global interruptions
    __bis_SR_register(GIE);
    // superLoop
    TB0CTL |= TBCLR;
    // variables that hold all the ticks
    while (true) {
        // configuration of the debug pin
        if (bt_has_message()) {
            bt_clear_flag();
            update_mt_with_bt(bt_get_data());
        }

        if (is_pos_update_ready_n_clean()) {

            // variables that hold the prev value;
            static int32_t prev_left_encoder = 0;
            static int32_t prev_right_encoder = 0;
            // atomic read
            __bic_SR_register(GIE);
            int32_t left_encoder = enc_get_left();
            int32_t right_encoder = enc_get_right();
            __bis_SR_register(GIE);
            update_odometry(&robot_info.pose, left_encoder - prev_left_encoder,
                            right_encoder - prev_right_encoder);

            prev_left_encoder = left_encoder;
            prev_right_encoder = right_encoder;
            // planning the next step
            desired_pos = global_planer(robot_info);
            desired_vel = local_planer(robot_info.pose, desired_pos);
            // sending the bluetooth data
            bt_send_data(&robot_info.pose, 10);
            debug_main_loop = false;
        }
        if (debug_main_loop) { // pin indicating that the main loop is not as quick as it need to be
            P1OUT ^= BIT0;
        }
        // navegation and control
        if (is_control_update_ready_n_clean()) {
            // variables that hold the prev value;
            static int32_t prev_left_encoder = 0;
            static int32_t prev_right_encoder = 0;
            // atomic read
            __bic_SR_register(GIE);
            int16_t left_encoder = enc_get_left();
            int16_t right_encoder = enc_get_right();
            __bis_SR_register(GIE);

            // update_control(desired_vel.linear_vel, desired_vel.ang_vel, left_encoder -
            // prev_left_encoder,
            ////             right_encoder - prev_right_encoder);

            prev_left_encoder = left_encoder;
            prev_right_encoder = right_encoder;
        }
    }
    return 0;
}

void config_ports(void) {
    config_mt();
    config_bt();
    config_ds();
    config_odometry();
    config_task_timer();
}
