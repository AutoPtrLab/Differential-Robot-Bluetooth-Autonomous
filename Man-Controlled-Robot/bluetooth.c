#include <msp430.h>
#include <stdint.h>
// bluetooth.c
#include "bluetooth.h"
#include "msp430f5529.h"
volatile bool bt_rx_flag = false;
volatile char bt_rx_buffer[BT_BUF_SIZE] = {
    '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0'}; // registers that holds the new data
volatile uint8_t bt_index = 0;
// configuration of the bluetooth module
void config_bt() {
  P3SEL |= BIT3 | BIT4;
  UCA0CTL1 |= UCSWRST; // enable software reset so we can change the registers

  UCA0CTL0 &= ~(UCPEN | UC7BIT | UCSPB |
                UCSYNC); // deactivate the parity , make sure is an 8 bit word
                         // and one stop bit ( 8-N-1), async mode
  UCA0CTL0 |= UCMODE_0;  // UART mode

  UCA0CTL1 |= UCSSEL_2; // SubMain clock (1Mhz)

  // setting the baud rate to 9600

  UCA0BR0 = 109;
  UCA0BR1 = 0;
  UCA0MCTL = UCBRS_2;

  UCA0CTL1 &= ~UCSWRST; // deactive the software lock
  UCA0IFG &= ~UCRXIFG;  // clean int flag
  UCA0IE |= UCRXIE;     // receiving interrupt
}

// ISR when the HC-05 receives something, remeber to update when we send and
// receive
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
  // if the character is # the message has ended and we activate the flag so we
  // can read it
  P1OUT ^= BIT0;
  char new_data = UCA0RXBUF; // read the internal buffer
  if (bt_index > BT_BUF_SIZE) {
    bt_index = 0;
  }
  if (new_data == END_CHAR) {

    bt_index = 0;
    bt_rx_flag = true;
    // else it just stores another character in the buff
  } else {
    bt_rx_buffer[bt_index] = new_data;
    bt_index++;
  }
  UCA0IFG &= ~UCRXIFG;
  // we clean the flag
}
