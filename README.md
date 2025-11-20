# I²C Voltage Sensor

## 1. Overview  

This device is a small voltage sensor built on the STM32C011J6 microcontroller.
It measures an analog voltage using the on-chip ADC, applies averaging and a calibration offset, and exposes the resulting millivolt reading over an I²C interface.

## 2. Usage

### I²C Address

* **7‑bit I²C address:** `0x24`
* Write byte: `0x48` (`0x24 << 1 | 0`)
* Read byte: `0x49` (`0x24 << 1 | 1`)

### Supported Commands

| Command                    | Code | Direction      | Description                                      |
| -------------------------- | ---- | -------------- | ------------------------------------------------ |
| `I2C_CMD_START_CONVERSION` | 0x01 | Master → Slave | Start a new ADC measurement + averaging          |
| `I2C_CMD_GET_STATUS`       | 0x02 | Master ← Slave | Read 1 byte: `0x00` = busy, `0x01` = ready       |
| `I2C_CMD_GET_RESULT`       | 0x03 | Master ← Slave | Read 2 bytes: 16‑bit unsigned mV (**MSB first**) |
| `I2C_CMD_CALIBRATE`        | 0x04 | Master → Slave | Measure input and store as flash offset          |

### Typical Usage Flow
1. Optional: Calibrate the device
    1. Short Vin+ to GND
    2. issue the CALIBRATE command.
    3. calibration offset is now stored in flash forever.

2. Send `0x01` to start conversion
3. Poll `0x02` until it returns `0x01` (ready)
4. Send `0x03` and read 2‑byte mV value

## 3. Example code

**Optional: Calibrate**

```c
cmd = I2C_CMD_CALIBRATE;
HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
```

This stores the current measured voltage as an offset in flash. You can short Vin+ to GND and issue this command to calibrate an offset for the device. This is stored in flash and persists between resets and power down.


**1. Start a conversion**

```c
uint8_t cmd = I2C_CMD_START_CONVERSION;
HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
```

**2. Poll status until ready**

```c
uint8_t status = 0;
cmd = I2C_CMD_GET_STATUS;

while (status == 0) {
    uint8_t cmd = I2C_CMD_GET_STATUS;
    HAL_I2C_Master_Transmit(&hi2c1, (36 << 1), &cmd, 1, HAL_MAX_DELAY); // Request status
    HAL_I2C_Master_Receive(&hi2c1, (36 << 1), &status, 1, HAL_MAX_DELAY); // Get Status
    HAL_Delay(10); // Small delay before polling again
}
```

**3. Read the result (mV)**

```c
uint8_t buf[2];
uint8_t cmd = I2C_CMD_GET_RESULT;

HAL_I2C_Master_Transmit(&hi2c1, 0x48, &cmd, 1, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1,  0x48, buf, 2, HAL_MAX_DELAY);

uint16_t vin_mv = ((uint16_t)buf[0] << 8) | buf[1];  // MSB first
```


## Firmware Structure

Project is generated with STM32CubeIDE, uses STM32 HAL.

Main project files:

* `Core/Src/main.c`
* `Core/Src/i2c_slave.c`
* `Core/Inc/i2c_slave.h`


## License

* ST HAL / CMSIS: see licenses in `Drivers/` and `CMSIS/`
* Application code: MIT
