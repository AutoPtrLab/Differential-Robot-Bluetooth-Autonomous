
#include "I2C_helper.h"
#include "odometry.h"
#include <stdint.h>

#include "bluetooth.h"
#include "distance_sensor.h"

#include "task_timer.h"

#include "motor.h"
#include "msp430f5529.h"
volatile uint16_t reloj = 0;
// foreground - background
// maybe add low power modes to optimise
int dist = 1000; // dist is in mm

Position *ROBOT_DEBUG_POS;
int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
    P1DIR |= BIT0;
    config_mt();
    config_bt();
    config_ds();
    config_odometry();
    config_task_timer();
    // config_I2C();

    // while(1){}
    __bis_SR_register(GIE);
    // superLoop
    // mt_forward();
    while (true) {
        reloj = TB0R;
        if (bt_has_message()) {

            update_mt(bt_get_data());
            bt_clear_flag();
        }
        if (ds_front_is_ready()) {
            dist = (ds_get_front_time() * 10) / 58;

            if (dist < 50) {

                // mt_stop();
            }
        }
        if (is_pos_update_ready_n_clean()) {
            ROBOT_DEBUG_POS = update_odometry();
            bt_send_data(ROBOT_DEBUG_POS,
                         12); // update the odometry and send it to the master device
        }
    }
    return 0;
}
