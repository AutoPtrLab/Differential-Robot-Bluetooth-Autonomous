
// ==========BLUETOOTH=======
// Transmitter P3.4
// Receiver P3.3

// the module this project makes uses is a HC-05  with 9600 baudrate and <8 N 1>
// the protocol we are going to use is simple, a group of chars that always ends
// in the '#' character ex : WF#
//
//
// The transmission is simplier, we just sent in 8bit pack the info the
// simulation needs in this case
//
// xpos,ypos , rad and the pythons script does the rest
#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "config.h"
#include <stdbool.h>
#include <stdint.h>
//( 8-N-1), async mode
//
// the first byte the micro sends so the master devices can receive the message
#define START_BYTE 0xAA

// bytes that the protocol needs to send message

#define PROTOCOL_TX_BYTES 2
extern volatile uint8_t
    _bt_tx_buffer[DATA_COUNT + PROTOCOL_TX_BYTES]; // copy of the info the bt needs to send

extern volatile bool _bt_rx_flag;                      // flag indicating there is a new word in
                                                       // the data register
extern volatile uint8_t _bt_rx_buffer[BT_RX_BUF_SIZE]; // registers that holds the new data
extern volatile bool _is_tx_ready; // flag describing the status of the sending process

// inline functions to improve scalation
static inline bool bt_has_message(void) { return _bt_rx_flag; }
// return a pointer to the first element of the array
static inline char *bt_get_data(void) { return (char *)_bt_rx_buffer; }
// clears the rx flag
static inline void bt_clear_flag(void) { _bt_rx_flag = false; }
// return if the tx is ready to be sent
static inline bool bt_tx_is_ready(void) { return _is_tx_ready; }

// configures the bluetooth module
void config_bt(void);

// sends the signal to communicate with the master device

// sends a message of x bytes to the receiver, if the previous message hasnt been completly sent
// it just omits the new one, to use this function is recommend to packed the complex data
// types so the padding doesnt disrupts the message, the function sends first the start byte and
// second and third the len of the message(not counting the protocol bytes),the functions also
// manages  the interruptions  so the flags and IE register shouldnt be used

void bt_send_data(const void *data, uint8_t total_bytes, uint8_t start_bytes);

#endif /* BLUETOOTH_H_ */
