
// ==========BLUETOOTH=======
// Transmitter P3.3
// Receiver P3.4

// the module this project makes uses is a HC-05  with 9600 baudrate and <8 N 1>
// the protocol we are going to use is simple, a group of chars that always ends
// in the '#' character ex : WF#
#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "config.h"
#include <stdbool.h>
#include <stdint.h>
//( 8-N-1), async mode

extern volatile bool bt_rx_flag; // flag indicating there is a new word in
                                 // the data register
extern volatile char
    bt_rx_buffer[BT_BUF_SIZE];    // registers that holds the new data
extern volatile uint8_t bt_index; // index of the current data

// inline functions to improve scalation
static inline bool bt_has_message(void) { return bt_rx_flag; }
// return a pointer to the first element of the array
static inline char *bt_get_data() { return (char *)bt_rx_buffer; }
// clears the rx flag
static inline void bt_clear_flag(void) { bt_rx_flag = false; }

// configures the bluetooth module
void config_bt();

#endif /* BLUETOOTH_H_ */
