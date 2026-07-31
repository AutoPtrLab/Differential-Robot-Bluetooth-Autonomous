
#include "bluetooth.h"
#include "distance_sensor.h"
#include "motor.h"
#include "msp430f5529.h"

// foreground - background
// maybe add low power modes to optimise
int dist = 1000; // dist is in mm
int main(void) {
  WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
  //__delay_cycles(5000000);
  TB0CTL = TBCLR | MC_2 | TBSSEL_2 | ID_2 | TBIE;
  P4DIR |= BIT7;
  P1DIR |= BIT0;
  config_mt();
  config_bt();
  config_ds();
  __bis_SR_register(GIE);
  // superLoop
  mt_forward();
  while (true) {
    if (bt_has_message()) {

      P1OUT &= ~BIT0;
      update_mt(bt_get_data());
      bt_clear_flag();
    }
    if (ds_front_is_ready()) {
      dist = (ds_get_front_time() * 10) / 58;

      if (dist < 50) {

        mt_break();
      }
    }
  }
  return 0;
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void ISR(void) {

  if (TBIFG & TB0CTL) {

    ds_trig_front();
    TB0CTL &= ~TBIFG;
  }
}
