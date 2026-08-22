#include "I2C_helper.h"
#include "msp430f5529.h"

// configures the pins and the conection with the MPU6050
void config_I2C(void) {
    P3SEL |= BIT0 | BIT1; // selecting the usci mode of the pin

    // pull up internal resistors
    P3REN |= BIT0 | BIT1;
    P1OUT |= BIT0 | BIT1;
    //----IMU-----
    UCB0CTL1 |= UCSWRST | UCSSEL_2 | UCTR; // software reset , submain clock(1MHz)
    // , activating transmitting mode
    UCB0BR0 = 5; // Setting up the divisor to have a 200kHz
                 // communication

    UCB0CTL0 &= ~(UCA10 | UCSLA10); // setting the 7 bit adress in the master and slave

    UCB0CTL0 |= UCMST | UCMODE_3 | UCSYNC; // set the microcontroller as the master ,
    // select I²C mode.select synchronous mode
    UCB0IFG &= ~UCRXIFG; // clean the int flag

    UCB0I2CSA = MPU_DIRECTION;

    UCB0CTL1 &= ~(UCSWRST); // deactivating the safe mode
}

// // starts the communication with the MPU6050
// void _start_communication(void) {
//     UCB0CTL1 |= UCTR;    // transitter
//     UCB0CTL1 |= UCTXSTT; // sends the start pulse

//     while (UCB0CTL1 & UCTXSTT) {
//         // waits the start pulse is sent(this happends when the UCTXSTT gets cleared
//         // by the hardware)
//     }
// }

// ends the communication sending the stop pulse to the MPU, this sends the NACK
// // and the stop
// void _stop_communication(void) {
//     UCB0CTL1 |= UCTXSTP;
//     while (UCB0CTL1 & UCTXSTP) {
//         // waits until the stop signal is sent
//     }
// }

// receives the memory address of the register it wants to read and returns its
// value , this fucntion should only be use if you only want one value and then
// ended the communication
uint8_t I2C_read_data(uint8_t address) {
    //_start_communication();

    UCB0CTL1 |= (UCTR | UCTXSTT); // transmitter mode and start pulse

    while (!(UCB0IFG & UCTXIFG)) {
        // waits until the buffer is empty
    }
    // UCB0CTL1 |= UCTR;    // transmitter mode
    UCB0TXBUF = address; // send the address
    while (!(UCB0IFG & UCTXIFG)) {
        // waits until the buffer is empty
    }

    // UCB0IFG &= ~UCTXIFG; // clean the transmitter flag
    UCB0CTL1 &= ~UCTR;   // receiver mode
    UCB0CTL1 |= UCTXSTT; // we send the restart
    while (UCB0CTL1 & UCTXSTT) {
        // waits for the restart
    }
    //  _stop_communication(); // send the stop mid read
    while (!(UCB0IFG & UCRXIFG)) {
        // waits until the receiver buffer holds a complete character
    }

    //  UCB0IFG &= ~UCRXIFG; // clean the receiver flag

    return (uint8_t)UCB0RXBUF;
}

// this function reads multiple registers of the MPU without ending the
// communication,receives the first address , the array where it will send the
// information and the number of words it wants to read(therefore the length of
// the array)
void I2C_read_mult_data(uint8_t first_address, uint8_t *arr, uint8_t length) {
    // comprobations
    if (length <= 0) {
        return;
    }

    UCB0CTL1 |= (UCTR | UCTXSTT); // transmitter mode and start pulse
    while (!(UCB0IFG & UCTXIFG)) {
        // wait for the buffer
    }
    //_start_communication();
    UCB0TXBUF = first_address;
    while (!(UCB0IFG & UCTXIFG)) {
        // waits until the buffer is empty
    }
    // UCB0IFG &= ~UCTXIFG; // clean the transmitter flag
    UCB0CTL1 &= ~UCTR;   // receiver mode
    UCB0CTL1 |= UCTXSTT; // sends the restart pulse

    while (UCB0CTL1 & UCTXSTT) {
        // waits restart pulse
    }
    // we read all the data minus one
    for (uint8_t i = 0; i < (length - 1); i++) {
        while (!(UCB0IFG & UCRXIFG)) {
            // waits until the receiver buffer holds a complete character
        }
        // if its the second last bit we send the stop signal
        // if (i == (length - 2)) {
        //     UCB0CTL1 |= UCTXSTP;
        // }
        arr[i] = UCB0RXBUF; // we store the buffer in the array
                            // UCB0IFG &= ~UCRXIFG; // clean the receiver flag
    }
    UCB0CTL1 |= UCTXSTP;
    //_stop_communication(); // stops the communication mid last bit

    // reads the last bit
    while (!(UCB0IFG & UCRXIFG)) {
        // waits until the receiver buffer holds a complete character
    }
    //  UCB0IFG &= ~UCRXIFG;         // clean the receiver flag
    arr[length - 1] = UCB0RXBUF; // we store the buffer in the array
}

void I2C_write_data(uint8_t address, uint8_t data) {
    UCB0CTL1 |= (UCTR | UCTXSTT); // transmitter mode and send start pulse

    while (!(UCB0IFG & UCTXIFG)) {
        // wait for the buffer
    }

    UCB0TXBUF = address;
    while (!(UCB0IFG & UCTXIFG)) {
        // wait for the buffer
    }
    // we write the data into that register
    UCB0TXBUF = data;
    while (!(UCB0IFG & UCTXIFG)) {
        // wait for the buffer
    }
    UCB0CTL1 |= UCTXSTP;
    while (!(UCB0IFG & UCTXIFG)) {
        // wait for the buffer
    }
}
