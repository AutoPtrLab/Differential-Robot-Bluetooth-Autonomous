//  helper for the I2C connection with the IMU MPU6050 and the launchpad
//  MSP430F5529
//
// The clock signal (SCL) needs to connect to the P3.1
// The data channel (SDA) needs to connect to the P3.0
//
// The communication function of this file are all blocking since the time
// necessary to read the 12 registers is not much and its simplifies a lot

#ifndef I2C_HELPER
#define I2C_HELPER

#include "msp430f5529.h"
#include <msp430.h>
#include <stdint.h>

// ======== PORT DEFINES ==========
#define MPU_DIRECTION 0x68 // normally the default direction of the MPU
#define PWR_MGMT_1 0x6B

// Accelerometer
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// Gyroscope
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

// DECLARATION

// configs the I2C ports
void config_I2C(void);
// sends the start pulse
void _start_communication(void);
// sends the stort and NACK pulse
void _stop_communication(void);
// reads one byte register and returns it
uint8_t I2C_read_data(uint8_t address);
// reads length adresses beginning with the first_address and stores them into the arr
void I2C_read_mult_data(uint8_t first_address, uint8_t *arr, uint8_t length);

void I2C_write_data(uint8_t address, uint8_t data);

#endif
