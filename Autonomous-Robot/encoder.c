#include "encoder.h"
#include "msp430f5529.h"
#include <stdint.h>

// Left encoder 2.7
// Right encoder 1.6
//
volatile uint16_t _enc_left_cont = 0;
volatile uint16_t _enc_right_cont = 0;

void config_enc() {
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
}

#pragma vector = PORT1_VECTOR
__interrupt void port1_ISR() {
  if (P1IFG & BIT6) {
    _enc_right_cont++;
    P1IES ^= BIT6;  // we toggle the flank selection so it detecs the rise and
                    // fall of the pulse
    P1IFG &= ~BIT6; // clean the int flag
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void port2_ISR() {
  if (P2IFG & BIT7) {
    _enc_left_cont++;
    P2IES ^= BIT7;  // we toggle the flank selection so it detecs the rise and
                    // fall of the pulse
    P2IFG &= ~BIT7; // clean the int flag
  }
}
