#include "bluetooth.h"

#include "motor.h"
#include <msp430.h>
//==============================
// function forward declarations
//==============================

// foreground - background
// maybe add low power modes to optimise
int main(void) {
  WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
  P1DIR |= BIT0;
  config_mt();
  config_bt();
  __bis_SR_register(GIE);
  while (true) {
    if (bt_has_message()) {

      P1OUT &= ~BIT0;
      update_mt(bt_get_data());
      bt_clear_flag();
    }
  }
  return 0;
}

#pragma vector = PORT1_VECTOR
__interrupt void EncoderLeft(void) {}
