# I²C Voltage Sensor (STM32C0)

Small STM32C0-based I²C "voltage sensor" slave. It measures an analog input with the on-chip ADC, averages the result, applies a calibration offset stored in flash, and exposes the final value in millivolts over I²C.

Target MCU: **STM32C011J6**
Project: generated with **STM32CubeIDE**, uses **STM32 HAL**

---

## I²C Interface (How to Use It)

### Address

* **7-bit I²C address:** `0x24`
* On the wire:

  * Write: `0x48` (`0x24 << 1 | 0`)
  * Read:  `0x49` (`0x24 << 1 | 1`)
* In STM32 HAL, the slave is configured with:

  ```c
  hi2c1.Init.OwnAddress1 = 0x48;  // 72 decimal
  ```

### Commands

All commands are **1‑byte** values sent from the master.
The typical flow is:

1. Start a conversion
2. Poll status
3. Read the result (16‑bit mV)
4. Optionally run calibration

#### Command Codes

```c
#define I2C_CMD_START_CONVERSION   0x01   // master → slave
#define I2C_CMD_GET_STATUS         0x02   // master ← slave (1 byte)
#define I2C_CMD_GET_RESULT         0x03   // master ← slave (2 bytes)
#define I2C_CMD_CALIBRATE          0x04   // master → slave
```

#### Protocol Summary

| Command                    | Code | Direction      | Description                                      |
| -------------------------- | ---- | -------------- | ------------------------------------------------ |
| `I2C_CMD_START_CONVERSION` | 0x01 | Master → Slave | Start a new ADC measurement + averaging          |
| `I2C_CMD_GET_STATUS`       | 0x02 | Master ← Slave | Read 1 byte: `0x00` = busy, `0x01` = ready       |
| `I2C_CMD_GET_RESULT`       | 0x03 | Master ← Slave | Read 2 bytes: 16‑bit unsigned mV (**MSB first**) |
| `I2C_CMD_CALIBRATE`        | 0x04 | Master → Slave | Measure input and store as flash offset          |

#### Typical Transaction Sequence

**1. Start a conversion**

```c
uint8_t cmd = I2C_CMD_START_CONVERSION;
HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
```

**2. Poll status until ready**

```c
uint8_t status = 0;
cmd = I2C_CMD_GET_STATUS;

do {
    HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1,  0x48, &status, 1, HAL_MAX_DELAY);
} while (status == 0x00);
```

**3. Read the result (mV)**

```c
uint8_t buf[2];
cmd = I2C_CMD_GET_RESULT;

HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1,  0x48, buf, 2, HAL_MAX_DELAY);

uint16_t vin_mv = ((uint16_t)buf[0] << 8) | buf[1];  // MSB first
```

**4. Optional: Calibrate**

```c
cmd = I2C_CMD_CALIBRATE;
HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
```

This stores the current measured voltage as an offset in flash.

---

## Features

* I²C slave at 7‑bit address **0x24**
* ADC measurement on `ADC1` channel 8
* 50-sample averaging and mV scaling
* Flash‑backed zero‑offset calibration
* Simple 1‑byte command protocol

---

## Firmware Structure

Main project files:

* `Core/Src/main.c`
* `Core/Src/i2c_slave.c`
* `Core/Inc/i2c_slave.h`

### Shared State (`i2c_slave.h`)

```c
extern volatile uint8_t i2c_calibration_request;
extern volatile uint8_t i2c_conversion_request;
extern volatile uint8_t i2c_conversion_busy;
extern volatile uint8_t i2c_data_ready;
extern volatile uint8_t i2c_tx_buf[2];   // measurement in mV, MSB first
```

### I²C Implementation (`i2c_slave.c`)

The slave uses STM32 HAL interrupt callbacks:

* `HAL_I2C_AddrCallback` — detect Tx/Rx direction & command
* `HAL_I2C_SlaveRxCpltCallback` — receive command and set flags
* `HAL_I2C_SlaveTxCpltCallback` — clear `i2c_data_ready`

### Measurement & Calibration (`main.c`)

* On `i2c_conversion_request`:

  * Take 50 ADC samples
  * Average → convert to mV → subtract stored offset
  * Store in `i2c_tx_buf[0..1]` (MSB first)
  * Set `i2c_data_ready = 1`

* On `i2c_calibration_request`:

  * Take 64 samples and convert to mV
  * Save as calibration offset in the last flash page
  * Future measurements subtract this value

---

## Building

1. Open **STM32CubeIDE**
2. Import this folder as an existing STM32 project
3. Ensure target MCU = **STM32C011J6**
4. Build and flash

---

## License

* ST HAL / CMSIS: see licenses in `Drivers/` and `CMSIS/`
* Application code: license as you prefer (MIT/BSD/etc.)
