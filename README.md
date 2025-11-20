# I2C Voltage Sensor (STM32C0)

Small STM32C0-based I²C "voltage sensor" slave device. It measures an analog input using the on-chip ADC, applies an adjustable calibration offset stored in flash, and exposes the result over an I²C slave interface.

The project is generated with STM32CubeIDE for the STM32C011J6 and uses the STM32 HAL.

## Features

- I²C slave at 7‑bit address `0x48` (`72` decimal)
- Single-ended ADC measurement on `ADC1` channel 8
- Averaging and scaling to compute input voltage in millivolts (mV)
- Flash-backed calibration offset for system-level zeroing
- Simple command-based I²C protocol

## Firmware Overview

Main files:

- `Core/Src/main.c`
- `Core/Src/i2c_slave.c`
- `Core/Inc/i2c_slave.h`

### Command Codes (`i2c_slave.h`)

```c
#define I2C_CMD_START_CONVERSION   0x01   // master → slave
#define I2C_CMD_GET_STATUS         0x02   // master ← slave (1 byte)
#define I2C_CMD_GET_RESULT         0x03   // master ← slave (2 bytes)
#define I2C_CMD_CALIBRATE          0x04   // master → slave
```

Shared state between the I²C driver and `main.c`:

```c
extern volatile uint8_t i2c_calibration_request;
extern volatile uint8_t i2c_conversion_request;
extern volatile uint8_t i2c_conversion_busy;
extern volatile uint8_t i2c_data_ready;
extern volatile uint8_t i2c_tx_buf[2];
```

### I²C Protocol (`i2c_slave.c`)

| Command                     | Code | Direction      | Description                                      |
|-----------------------------|------|----------------|--------------------------------------------------|
| `I2C_CMD_START_CONVERSION` | 0x01 | Master → Slave | Request a new ADC conversion and averaging       |
| `I2C_CMD_GET_STATUS`       | 0x02 | Master ← Slave | Read 1 byte: `0x00` = busy, `0x01` = ready       |
| `I2C_CMD_GET_RESULT`       | 0x03 | Master ← Slave | Read 2 bytes: 16‑bit unsigned mV (MSB first)     |
| `I2C_CMD_CALIBRATE`        | 0x04 | Master → Slave | Trigger offset calibration and flash save        |

The I²C logic uses the STM32 HAL callbacks (`HAL_I2C_AddrCallback`, `HAL_I2C_SlaveRxCpltCallback`, `HAL_I2C_SlaveTxCpltCallback`, `HAL_I2C_ListenCpltCallback`) to:

- Receive 1‑byte commands from the master
- Set `i2c_conversion_request` / `i2c_calibration_request`
- Serve status and measurement data from `i2c_tx_buf`

### Measurement & Calibration (`main.c`)

- On `i2c_conversion_request`:
	- Takes 50 ADC samples, averages them, converts to mV using a divider factor
	- Subtracts `adc_offset_mv` (calibration offset) and clamps to 16‑bit
	- Stores result in `i2c_tx_buf[0..1]` (MSB first) and sets `i2c_data_ready`

- On `i2c_calibration_request`:
	- Takes 64 ADC samples, converts to mV
	- Saves this value as `adc_offset_mv` in the last flash page
	- Future readings subtract this offset

## Building

Use STM32CubeIDE:

1. Open STM32CubeIDE.
2. Import the `I2C-Voltage-Sensor` folder as an existing project.
3. Build the project and flash/debug on an STM32C011J6 board.

## License

HAL/CMSIS components use their respective ST licenses (see `Drivers/` and `CMSIS/`). Application code can be licensed as you prefer.

