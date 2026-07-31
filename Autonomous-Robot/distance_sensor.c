#include "distance_sensor.h"

// Front Sensor:
//      Trig = P4.2
//      Echo = P2.5 / TA2.2
// Left Sensor:
//      Trig = P3.2
//      Echo = P2.4 / TA2.1
// Right Sensor:
//      Trig = P8.1
//      Echo = P2.3 / TA2.0

// auxiliar variables to capture the cycle of the clock in the beggining of the
// distance calculation
volatile uint16_t aux_front = 0;
volatile uint16_t aux_left = 0;
volatile uint16_t aux_right = 0;

// extern variables defined here
volatile uint16_t _ds_front_time = 0;
volatile uint16_t _ds_left_time = 0;
volatile uint16_t _ds_right_time = 0;

volatile bool _ready_front = false;
volatile bool _ready_left = false;
volatile bool _ready_right = false;

void config_ds() {
  // triggers
  P4DIR |= BIT2;
  P3DIR |= BIT2;
  P8DIR |= BIT1;

  // echoes
  P2DIR &= ~(BIT5 | BIT4 | BIT3);
  P2SEL |= BIT5 | BIT4 | BIT3;

  // timer TA2 configuration ,compare mode
  // since the maximun in can reach is 38 ms (when there is no obstacule)
  // we use the 1Mhz and still doesnt overflow
  TA2CTL &= ~TAIFG;                         // clean int flag
  TA2CTL |= TASSEL_2 | ID_0 | MC_2 | TACLR; // continuos mode and 1Mhz

  // capture both edges
  TA2CCTL0 &= ~CCIFG;
  TA2CCTL0 |= CM_3 | CAP | CCIE;
  TA2CCTL1 &= ~CCIFG;
  TA2CCTL1 |= CM_3 | CAP | CCIE;
  TA2CCTL2 &= ~CCIFG;
  TA2CCTL2 |= CM_3 | CAP | CCIE;
}

// interruption to capture how long was the electrical pulse of the echo pin

// the msp430 has two differnte vectores int , one for the compare and capture
// zero and other for the othe 3 ports
#pragma vector = TIMER2_A0_VECTOR
__interrupt void right_ISR(void) {
  if (TA2CCTL0 & CCI) {
    // the input is ON
    aux_right = TA2R;
  } // if the input is off the pulse has ended and we calculate how many secons
    // has lasted
  else {
    _ds_right_time = TA2R - aux_right;
    _ready_right = true; // ready to check
  }
  TA2CCTL0 &= ~CCIFG; // we clean the flag
}

// ISR of the CC2 and CC3
#pragma vector = TIMER2_A1_VECTOR
__interrupt void left_front_ISR(void) {

  // if the int comes form the front sensor
  if (TA2CCTL2 & CCIFG) {

    if (TA2CCTL2 & CCI) {
      // the input is ON
      aux_front = TA2R;
    } // if the input is off the pulse has ended and we calculate how many
      // secons has lasted
    else {
      P4OUT ^= BIT7;
      _ds_front_time = TA2R - aux_front;
      _ready_front = true; // ready to check
    }
    TA2CCTL2 &= ~CCIFG; // we clean the flag
  }
  // if the int comes from the left sensor
  if (TA2CCTL1 & CCIFG) {
    if (TA2CCTL1 & CCI) {
      // the input is ON
      aux_left = TA2R;
    } // if the input is off the pulse has ended and we calculate how many
      // secons has lasted
    else {
      _ds_left_time = TA2R - aux_left;
      _ready_left = true; // ready to check
    }
    TA2CCTL1 &= ~CCIFG; // we clean the flag
  }
}
