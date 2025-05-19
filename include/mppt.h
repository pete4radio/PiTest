#pragma once
#define LT8491_ADDR 0x10
#define I2C_TIMEOUT_MS 1000

// Configuration register addresses and values
typedef struct {
    uint8_t addr;
    uint16_t value;
} Config;

// Function prototypes
uint8_t init_mppt();
uint8_t do_mppt();