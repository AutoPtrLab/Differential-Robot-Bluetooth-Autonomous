#include "task_timer.h"
#include "distance_sensor.h"
#include "encoder.h"
#include "msp430f5529.h"
#include "stdbool.h"

void config_task_timer() {
  TB0CTL = TBCLR | MC_2 | TBSSEL_1 | ID_0 | TBIE; // continous mode, 32 768 Hz,

  // activating the interruptions of the compare blocks

  // TB0CCTL0 |= CCIE;
  TB0CCTL1 |= CCIE;
  TB0CCTL2 |= CCIE;
  TB0CCTL3 |= CCIE;
  // TB0CCTL4 |= CCIE;
  // TB0CCTL5 |= CCIE;
  //  TB0CCTL6 |= CCIE;

  // cleaning the flags
  TB0CCTL0 &= ~CCIFG;
  TB0CCTL1 &= ~CCIFG;
  TB0CCTL2 &= ~CCIFG;
  TB0CCTL3 &= ~CCIFG;
  TB0CCTL4 &= ~CCIFG;
  TB0CCTL5 &= ~CCIFG;
  TB0CCTL6 &= ~CCIFG;
  //  settings of the frequency of the task

  // front sensor
  TB0CCR1 = FRONT_DS_CYCLES;
  // left right sensors
  TB0CCR2 = LEFT_RIGHT_DS_CYCLES;
  // position update
  TB0CCR3 = POSITION_UPDATE_CYCLES;
}

volatile bool _is_update_pos_ready = false;

// ISR for the TB0CCR0
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR(void) {}

// ISR for the rest of the Control registers
#pragma vector = TIMER0_B1_VECTIR
__interrupt void task_controller(void) {
  // front distance sensor
  if (TB0CCTL1 & CCIFG) {
    ds_trig_front();
    TB0CCTL1 &= ~CCIFG; // cleaning the int flag
  }
  // right and left distance sensor
  if (TB0CCTL2 & CCIFG) {
    ds_trig_left();
    ds_trig_right();
    TB0CCTL2 &= ~CCIFG; // cleaning the int flag
  }
  // position update
  if (TB0CCTL3 & CCIFG) {
    _is_update_pos_ready = true;
    TB0CCTL3 &= ~CCIFG; // cleaning the int flag
  }
}
