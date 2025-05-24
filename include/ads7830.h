#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C address for ADS7830
#define ADS7830_I2C_ADDR 0x48

// Function prototypes
uint8_t init_ADC(i2c_inst_t *i2c);
uint8_t read_ADC(i2c_inst_t *i2c, uint8_t channel, uint16_t *adc_voltage);