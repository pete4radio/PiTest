#pragma once
// Timeout for I2C operations in milliseconds
#define I2C_TIMEOUT_MS_POWER 100
#define ADM1176_I2C_ADDR 0x4A

#define PICO_OK 0
#define PICO_ERROR_GENERIC 1

// Function prototypes
uint8_t init_power(i2c_inst_t *i2c);
uint8_t do_power(i2c_inst_t *i2c, float *voltage, float *current);