#include "odometry.h"
#include "I2C_helper.h"
#include "config.h"

#include "msp430f5529.h"
#include "trigonometry_helper.h"
#include <stdint.h>
//
// Left encoder 2.7
// Right encoder 1.6
//
//
volatile int16_t _enc_left_cont = 0;
volatile int16_t _enc_right_cont = 0;

volatile int8_t enc_left_dir = 0;
volatile int8_t enc_right_dir = 0;
Position _robot_pos = {0, 0, 0};

void config_odometry() {
  //-----ENCODER------
  // no need for an external resistance
  //
  // set the pins as entrance
  P1DIR &= ~BIT6;
  P2DIR &= ~BIT7;

  // clean int flags
  P1IFG &= ~BIT6;
  P2IFG &= ~BIT7;

  P1IES |= BIT6;
  P2IES |= BIT7;

  // activate the interrupts
  P2IE |= BIT7;
  P1IE |= BIT6;

  config_I2C(); // configures the connection with the IMU
}

void update_odometry() {
  __bic_SR_register(GIE); // off the ints.
  uint16_t enc_left = enc_get_left();
  uint16_t enc_right = enc_get_right();
  __bis_SR_register(GIE); // reactivation of the global ints
  // dx = enc_cont * mm_per_pulse
  // D = (dl+dr)/2
  uint32_t dist_left =
      enc_left * UM_PER_PULSE; // distance the left wheel travels
  uint32_t dist_right =
      enc_right * UM_PER_PULSE; // distance the right wheel travels
  uint32_t distance =
      (dist_left + dist_right) /
      2; // distance in micrometers taking in reference to the rotation center
  // rad = (dl + dr)/ (2*wheel_rot_center == wheel_wheel_dist)
  uint32_t rads = ((dist_right - dist_left) * 1000000) /
                  WHEEL_WHEEL_DIST_UM; // rads * 1000000

  _robot_pos.x +=
      (distance * urad_cos(_robot_pos.rad + (rads - _robot_pos.rad) / 2)) /
      10000;
  _robot_pos.y +=
      (distance * urad_sin(_robot_pos.rad + (rads - _robot_pos.rad) / 2)) /
      10000;
  _robot_pos.rad = rads;
}

#pragma vector = PORT1_VECTOR
__interrupt void port1_ISR() {
  if (P1IFG & BIT6) {
    _enc_right_cont += enc_right_dir;
    P1IES ^= BIT6;  // we toggle the flank selection so it detecs the rise and
                    // fall of the pulse
    P1IFG &= ~BIT6; // clean the int flag
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void port2_ISR() {
  if (P2IFG & BIT7) {
    _enc_left_cont += enc_left_dir;
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
