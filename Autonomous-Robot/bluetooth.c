// bluetooth.c
#include <msp430.h>
#include <stdint.h>

#include "bluetooth.h"
#include "config.h"

#include "msp430f5529.h"

volatile uint8_t _bt_tx_buffer[DATA_COUNT] = {0}; // trasmission buffer
// private tx index
static volatile uint8_t _bt_tx_index = 0;
static volatile bool _is_tx_ready = true; // flag describing the status of the sending process

static int16_t _bt_tx_bytes = 0; // how many bytes we have to send

// private rx index
static volatile uint8_t _bt_rx_index = 0;
volatile bool _bt_rx_flag = false;
volatile uint8_t _bt_rx_buffer[BT_RX_BUF_SIZE] = {
    '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0'}; // registers that holds the new data

// configuration of the Bluetooth module
void config_bt() {
    P3SEL |= BIT3 | BIT4;
    UCA0CTL1 |= UCSWRST; // enable software reset so we can change the registers

    UCA0CTL0 &= ~(UCPEN | UC7BIT | UCSPB | UCSYNC); // deactivate the parity , make sure is an 8 bit
                                                    // word and one stop bit ( 8-N-1), async mode
    UCA0CTL0 |= UCMODE_0;                           // UART mode

    UCA0CTL1 |= UCSSEL_2; // SubMain clock (1Mhz)

    // setting the baud rate to 9600

    UCA0BR0 = 109;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_2;

    UCA0CTL1 &= ~UCSWRST;            // deactive the software lock
    UCA0IFG &= ~(UCRXIFG | UCTXIFG); // clean int flag
    UCA0IE |= UCRXIE;                // receiving  interrupt

    _is_tx_ready = true; // activate the trasmission flag
}

void bt_send_data(const void *data, uint8_t total_bytes) {
    // if the tx isnt ready we omit the new data

    if (!_is_tx_ready) {
        return;
    }
    // if the total_bytes are larger than the array
    if (total_bytes + PROTOCOL_TX_BYTES > DATA_COUNT) {
        return;
    }
    _bt_tx_bytes = total_bytes + PROTOCOL_TX_BYTES;
    uint8_t *data_divider = (uint8_t *)data;
    _is_tx_ready = false;
    //  deativate the ints
    __bic_SR_register(GIE);
    _bt_tx_buffer[0] = START_BYTE;  // send start byte
    _bt_tx_buffer[1] = total_bytes; // send lenght of the message
    for (uint16_t i = 0; i < total_bytes; i++) {
        _bt_tx_buffer[i + PROTOCOL_TX_BYTES] = data_divider[i];
    }
    // reactivate the ints since the data has been copied
    __bis_SR_register(GIE);
    UCA0TXBUF = _bt_tx_buffer[0]; // we send the first byte
    UCA0IE |= UCTXIE; // activate the transmission interruptions that trigger every time the buffer
    // has been emptied
    _bt_tx_index = 1; // the index always starts pointing to the second position in the tx buffer
}

// void bt_send_message(const void *data, uint16_t total_bytes) {

//     // sending the head message to alert the other device
//     bt_send_data(, uint16_t total_bytes);

//     bt_send_data(data, length);
// }

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {

    switch (__even_in_range(UCA0IV, 4)) {
    case 0:
        break;

    case 2: // RX interruption
        // if the character is # the message has ended and we activate the flag so we
        // can read it
        {
            char new_data = UCA0RXBUF; // read the internal buffer

            if (_bt_rx_index > BT_RX_BUF_SIZE) {
                _bt_rx_index = 0;
            }

            if (new_data == END_CHAR) {
                _bt_rx_index = 0;
                _bt_rx_flag = true;
                // else it just stores another character in the buff
            } else {
                _bt_rx_buffer[_bt_rx_index] = new_data;
                _bt_rx_index++;
            }
        }
        break;

    case 4: // TX interruption
        // transmitter int ,
        //  the transmitter interrupt happens when the TXBUFF has been emptied
        if (_bt_tx_index >= _bt_tx_bytes) {
            _bt_tx_index = 0;

            // we have read the buffer ,know we get the newest info possible
            UCA0IE &= ~UCTXIE; // deactivation of the interruptions

            _is_tx_ready = true; // since the data has been passed we are available for more

        } else { // if we havent written the entire message then we continue
            UCA0TXBUF = _bt_tx_buffer[_bt_tx_index++];
        }
        break;

    default:
        break;
    }
}
