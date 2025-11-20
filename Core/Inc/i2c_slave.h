/*
 * i2c_slave.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Stian
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include <stdint.h>

/* I2C command codes */
#define I2C_CMD_START_CONVERSION   0x01   // master → slave
#define I2C_CMD_GET_STATUS         0x02   // master ← slave (1 byte)
#define I2C_CMD_GET_RESULT         0x03   // master ← slave (2 bytes)
#define I2C_CMD_CALIBRATE          0x04   // master -> slave

/* Shared state used by main.c */
extern volatile uint8_t i2c_calibration_request;
extern volatile uint8_t i2c_conversion_request;   // main loop starts ADC
extern volatile uint8_t i2c_conversion_busy;      // optional status flag
extern volatile uint8_t i2c_data_ready;           // main loop sets when result ready

extern volatile uint8_t i2c_tx_buf[2];            // holds MSB, LSB of result

/* Initialize safe slave driver */
void I2C_Slave_Init(void);

#endif /* INC_I2C_SLAVE_H_ */
