#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "power.h"

// Masks for data processing
#define DATA_V_MASK 0xF0
#define DATA_I_MASK 0x0F

// Buffers for I2C communication
static uint8_t _cmd[1];
static uint8_t _extcmd[2] = {0x00, 0x04};
static uint8_t _buffer[3];
static uint8_t _status[1];

// Sense resistor value
static float sense_resistor = 0.1;

// Initialize the ADM1176 device
uint8_t init_power(i2c_inst_t *i2c) {
    // Configure the ADM1176 for continuous voltage and current monitoring
    _cmd[0] = (1 << 0) | (1 << 2); // V_CONT and I_CONT
    absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS_POWER);
    int result = i2c_write_blocking_until(i2c, ADM1176_I2C_ADDR, _cmd, 1, false, timeout);
    return (result == PICO_ERROR_GENERIC) ? PICO_ERROR_GENERIC : PICO_OK;
}

// Read voltage and current from the ADM1176
uint8_t do_power(i2c_inst_t *i2c, float *voltage, float *current) {
    // Read data into the buffer
    absolute_time_t timeout = make_timeout_time_ms(I2C_TIMEOUT_MS_POWER);
    int result = i2c_read_blocking_until(i2c, ADM1176_I2C_ADDR, _buffer, 3, false, timeout);
    if (result == PICO_ERROR_GENERIC) {
        return PICO_ERROR_GENERIC;
    }

    // Process raw voltage and current data
    uint16_t raw_voltage = ((_buffer[0] << 8) | (_buffer[2] & DATA_V_MASK)) >> 4;
    uint16_t raw_current = (_buffer[1] << 4) | (_buffer[2] & DATA_I_MASK);

    // Convert raw data to voltage and current
    *voltage = (26.35 / 4096) * raw_voltage; // Voltage in volts
    *current = ((0.10584 / 4096) * raw_current) / sense_resistor; // Current in amperes

    return PICO_OK;
}