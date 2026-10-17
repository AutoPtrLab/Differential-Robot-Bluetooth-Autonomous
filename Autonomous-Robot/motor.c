#include "motor.h"
#include <ctype.h>
#include <stdint.h>

// configuration of the motor pins
void config_mt(void) {
    // sets as and exit AIN1 and AIN2
    P1DIR |= BIT4 | BIT5;
    // sets as and exit BIN1 and BIN2
    P2DIR |= BIT0 | BIT6;

    // we set as an special exit the PWM pins as they will be controlled by the
    // TA0
    P1DIR |= BIT2 | BIT3;
    P1SEL |= BIT2 | BIT3;

    // configuration of the timer
    TA0CTL |= MC_1 + TACLR + TASSEL_2 +
              ID_0; // we configure it to run with the sub_main_clock (1MHz) , up mode
    // P1.2 PWMA signal
    TA0CCTL1 |= OUTMOD_7;
    // P1.3 PWMB signal
    TA0CCTL2 |= OUTMOD_7;

    // the clock resets when the counters reachs 50, the velocity of the motor
   // changes with the duty cycles
    TA0CCR0 = 50;
    TA0CCR2 = 25;
    TA0CCR1 = 25;

    // prepare the STBY pin
    P8DIR |= BIT2;
   // we star with the syby
    P8OUT |= BIT2;
    // neutral configuration
    P1OUT &= ~BIT4;
    P1OUT &= ~BIT5;
    P2OUT &= ~BIT0;
    P2OUT &= ~BIT6;
}

// bluetooth motor control
void update_mt_with_bt(char *message) {

    switch (message[0]) {
    case 'L': // left MOTOR

        switch (message[1]) {
        case 'B': // backwards
            mt_left_backward();
            break;
        case 'N': // neutral
            P1OUT &= ~BIT4;
            P1OUT &= ~BIT5;
            break;
        case 'F': // forward
            mt_left_forward();
            break;

        case 'S': // stop ,active brake
            mt_left_stop();
            break;
        }
        break;
        //=========end of left motor Switch========
        //
    case 'R': // right motor
        switch (message[1]) {
        case 'B': // backwards
            mt_right_backward();
            break;
        case 'N': // neutral
            P2OUT &= ~BIT0;
            P2OUT &= ~BIT6;
            break;
        case 'F': // forward
            mt_right_forward();
            break;

        case 'S': // stop ,active brake
            mt_right_stop();
            break;
        } // end of right motor switch
        break;
    case 'C':                               // change velocity
        if (isdigit(message[1])) {          // We check if the message is a digit number
            uint8_t vel = message[1] - '0'; // Since we can only receive in the
                                            // protocol numbers this gives as the int
                                            //
            // vel is between 0 and 9 and since the max is 50 we add 5 as the
            // minimum vel
            TA0CCR2 = 5 * vel + 5;
            TA0CCR1 = 5 * vel + 5;
        }

        break;
    }
}
