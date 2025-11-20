/*
 * i2c_slave.c
 *
 *  Created on: Oct 9, 2025
 *      Author: Stian
 */

#include "main.h"
#include "i2c_slave.h"

extern I2C_HandleTypeDef hi2c1;

volatile uint8_t i2c_calibration_request = 0;
volatile uint8_t i2c_conversion_request = 0;
volatile uint8_t i2c_conversion_busy    = 0;
volatile uint8_t i2c_data_ready         = 0;

volatile uint8_t i2c_tx_buf[2];
volatile uint8_t i2c_rx_cmd = 0;

static uint8_t rx_byte;
static uint8_t zero      = 0;
static uint8_t zeros_2[2] = {0,0};
static uint8_t status_byte = 0;

static inline void i2c_listen_restart(void)
{
    HAL_I2C_EnableListen_IT(&hi2c1);
}

void I2C_Slave_Init(void)
{
    i2c_rx_cmd = 0;
    i2c_conversion_request = 0;
    i2c_conversion_busy    = 0;
    i2c_data_ready         = 0;

    i2c_listen_restart();
}

/* Address matched */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    if (hi2c->Instance != I2C1) return;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
        /* Master → slave: receive 1 command byte */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_byte, 1, I2C_NEXT_FRAME);
    }
    else
    {
        /* Master ← slave: respond based on last command */
        switch (i2c_rx_cmd)
        {
            case I2C_CMD_GET_STATUS:
                status_byte = (i2c_data_ready ? 1 : 0);
                HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &status_byte, 1,
                                              I2C_NEXT_FRAME);
                break;

            case I2C_CMD_GET_RESULT:
                if (i2c_data_ready)
                    HAL_I2C_Slave_Seq_Transmit_IT(hi2c,
                                                  (uint8_t *)i2c_tx_buf,
                                                  2,
                                                  I2C_NEXT_FRAME);
                else
                    HAL_I2C_Slave_Seq_Transmit_IT(hi2c,
                                                  zeros_2,
                                                  2,
                                                  I2C_NEXT_FRAME);
                break;

            default:
                HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &zero, 1,
                                              I2C_NEXT_FRAME);
                break;
        }
    }
}

/* 1-byte command received */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    i2c_rx_cmd = rx_byte;

    if (i2c_rx_cmd == I2C_CMD_START_CONVERSION)
    {
        i2c_conversion_request = 1;
        i2c_conversion_busy    = 1;
        i2c_data_ready         = 0;
    }

    if (i2c_rx_cmd == I2C_CMD_CALIBRATE)
    {
        i2c_calibration_request = 1;
    }

    /* Do NOT restart listen here – wait for STOP first */
}

/* transmit completed (status or result) */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    if (i2c_rx_cmd == I2C_CMD_GET_RESULT)
    {
        i2c_data_ready = 0;
    }

    /* Do NOT restart listen here – wait for STOP */
}

/* STOP / end of listen cycle */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    i2c_listen_restart();
}
