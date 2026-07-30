#include "motor.h"
#include "bluetooth.h"
#include "msp430f5529.h"
#include <ctype.h>
#include <msp430.h>
#include <stdint.h>

void config_mt() {
  // sets as and exit AIN1 and AIN2
  P1DIR |= BIT4 | BIT5;
  // sets as and exit BIN1 and BIN2
  P2DIR |= BIT0 | BIT6;

  // we set as an special exit the PWM pins as they will be controlled by the
  // TA0
  P1DIR |= BIT2 | BIT3;
  P1SEL |= BIT2 | BIT3;

  // configuration of the timer
  TA0CTL |=
      MC_1 + TACLR + TASSEL_2 +
      ID_0; // we configure it to run with the sub_main_clock (1MHz) , up mode
  // P1.2 PWMA signal
  TA0CCTL1 |= OUTMOD_7;
  // P1.3 PWMB signal
  TA0CCTL2 |= OUTMOD_7;

  // prepare the STBY pin
  P2DIR |= BIT2;
  // the clock resets when the counters reachs 50, the velocity of the motor
  // changes with the duty cycles
  TA0CCR0 = 50;

  // Buzzer configuration
  P2DIR |= BIT4;
  P2SEL |= BIT4;
  TA2CTL |= MC_1 + TACLR + TASSEL_2 + ID_0; // 1 Mhz
  TA2CCTL2 |= OUTMOD_7;
  TA2CCR2 = 0;
  TA2CCR0 = 520;
  // we star with the syby
  P2OUT |= BIT2;
}

void update_mt(char *message) {

  switch (message[0]) {
  case 'L': // left MOTOR

    switch (message[1]) {
    case 'B': // backwards
      P1OUT |= BIT5;
      P1OUT &= ~BIT4;
      break;
    case 'N': // neutral
      P1OUT &= ~BIT4;
      P1OUT &= ~BIT5;
      break;
    case 'F': // forward
      P1OUT |= BIT4;
      P1OUT &= ~BIT5;
      break;

      break;
    case 'S': // stop ,active brake
      P1OUT |= BIT5;
      P1OUT |= BIT4;
      break;
    }
    break;
    //=========end of left motor Switch========
    //
  case 'R': // right motor
    switch (message[1]) {
    case 'B': // backwards
      P2OUT |= BIT6;
      P2OUT &= ~BIT0;
      break;
    case 'N': // neutral
      P2OUT &= ~BIT0;
      P2OUT &= ~BIT6;
      break;
    case 'F': // forward
      P2OUT |= BIT0;
      P2OUT &= ~BIT6;
      break;

    case 'S': // stop ,active brake
      P2OUT |= BIT6;
      P2OUT |= BIT0;
      break;

      break;
    } // end of right motor switch

  case 'C':                    // change velocity
    if (isdigit(message[1])) { // we check if the message is a digit number
      uint8_t vel = message[1] - '0'; // since we can only receive in the
                                      // porotoclo numbers this gives as the int
                                      //
      // vel is between 0  and 9 and sicne the max is 50 we add 5 as tghe
      // minimum vel
      TA0CCR2 = 5 * vel + 5;
      TA0CCR1 = 5 * vel + 5;
    }

    break;
  case 'W':
    TA2CCR2 = 250;
    break;
  case 'S':
    TA2CCR2 = 0;
  }
}
