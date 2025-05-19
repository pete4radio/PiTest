#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pins.h"
#include "mppt.h"

Config CFG[] = {
    {0x28, 0x2710},
    {0x2A, 0x0BC2},
    {0x2C, 0x0CE4},
    {0x2E, 0x3854},
    {0x30, 0x0604},
    {0x32, 0x0942},
    {0x34, 0x0728},
    {0x36, 0x02DC},
    {0x38, 0x03B9},
};

// Telemetry register addresses
uint8_t TELE[] = {
    0x00,
    0x02,
    0x04,
    0x08,
    0x0A,
};
static uint8_t already_init = 0;  // Flag to indicate if the MPPT has been initialized

uint8_t init_mppt() {
    if (already_init == 0)
    {
    // Initialize I2C on the default pins
        // i2c_init(i2c0, 100000);
        // gpio_set_function(SAMWISE_MPPT_SDA_PIN, GPIO_FUNC_I2C);
        // gpio_set_function(SAMWISE_MPPT_SCL_PIN, GPIO_FUNC_I2C);
        // gpio_pull_up(SAMWISE_MPPT_SDA_PIN);
        // gpio_pull_up(SAMWISE_MPPT_SCL_PIN);
        // sleep_ms(100);  // Wait for the I2C bus to stabilize

        // Test I2C connection
        uint8_t result[1];
        uint8_t ctrl_chrg_en_addr = 0x23;
        if (i2c_write_blocking_until(i2c0, LT8491_ADDR, &ctrl_chrg_en_addr, 1, true, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
            return 1;  // Error
        }
        if (i2c_read_blocking_until(i2c0, LT8491_ADDR, result, 1, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
            printf("CTRL_CHRG_EN: %d\n", result[0]);
            return 1;  // Error
            }

    // Write configuration values to LT8491
        for (size_t i = 0; i < sizeof(CFG) / sizeof(CFG[0]); i++) {
            uint8_t addr = CFG[i].addr;
            uint16_t value = CFG[i].value;
            uint8_t low_byte = value & 0xFF;
            uint8_t high_byte = (value >> 8) & 0xFF;

            uint8_t out_buffer[] = {addr, low_byte};
            uint8_t out_buffer_2[] = {addr + 1, high_byte};

            if (i2c_write_blocking(i2c0, LT8491_ADDR, out_buffer, sizeof(out_buffer), false) == PICO_ERROR_GENERIC ||
                i2c_write_blocking(i2c0, LT8491_ADDR, out_buffer_2, sizeof(out_buffer_2), false) == PICO_ERROR_GENERIC) {
                return 1;  // Error
            }

            uint8_t result_2[2];
            if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
                i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
                return 1;  // Error
            }

            printf("%02x: %04x\n", addr, (result_2[1] << 8 | result_2[0]));
        }
        already_init = 1;  // Set the flag to indicate initialization is complete
        printf("MPPT.c initialized\n");
        }
    return 0;  // Success
}

uint8_t do_mppt() {
    uint8_t result[1];
    uint8_t result_2[2];

    uint8_t ctrl_chrg_en_addr = 0x23;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &ctrl_chrg_en_addr, 1, true) != PICO_ERROR_GENERIC &&
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result, 1, false, make_timeout_time_ms(10)) != PICO_ERROR_GENERIC) {
        printf("CTRL_CHRG_EN: %d\n", result[0]);
    } else {
        return 1;  // Error
    }

    uint8_t addr = 0x12;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result, 1, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("STAT_CHARGER: %d\n", result[0]);

    addr = 0x19;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result, 1, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("STAT_CHRG_FAULTS: %d\n", result[0]);

    // Output Telemetry
    addr = 0x08;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_IOUT: %d mA\n", result_2[0] | (result_2[1] << 8));

    addr = 0x0C;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_VBAT: %d V\n", (result_2[0] | (result_2[1] << 8)) / 100);

    addr = 0x02;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_POUT: %d W\n", (result_2[0] | (result_2[1] << 8)) / 100);

    addr = 0x0A;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_IIN: %d mA\n", result_2[0] | (result_2[1] << 8));

    addr = 0x10;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_VINR: %d V\n", (result_2[0] | (result_2[1] << 8)) / 100);

    addr = 0x0E;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_VIN: %d V\n", (result_2[0] | (result_2[1] << 8)) / 100);

    addr = 0x04;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_PIN: %d W\n", (result_2[0] | (result_2[1] << 8)) / 100);

    addr = 0x06;
    if (i2c_write_blocking(i2c0, LT8491_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC ||
        i2c_read_blocking_until(i2c0, LT8491_ADDR, result_2, 2, false, make_timeout_time_ms(10)) == PICO_ERROR_GENERIC) {
        return 1;  // Error
    }
    printf("TELE_EFF: %d %%\n", (result_2[0] | (result_2[1] << 8)) / 100);

    return 0;  // Success
}