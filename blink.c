/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
#include "pico/time.h"

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 900
#endif

#define PIN_SDA 4
#define PIN_SCL 5

bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

int pico_I2C_init(void) {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main() {
    int counter = 0;
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    rc = pico_I2C_init();
    hard_assert(rc == PICO_OK);

    //Initialize chosen serial port
    stdio_init_all();
    //  see https://forums.raspberrypi.com/viewtopic.php?t=300136
    int i = 100;
//   while (!tud_cdc_connected() && i--) { sleep_ms(100);  }
//    printf("USB_connected or timed out\n");

//  LED ON
    absolute_time_t previous_time_LED_ON = get_absolute_time();     // ms
    uint32_t interval_LED_ON = 1000;

// LED OFF
    absolute_time_t previous_time_LED_OFF = get_absolute_time();     // ms
    uint32_t interval_LED_OFF = 500;

//I2C Scan
    absolute_time_t previous_time_I2C = get_absolute_time();     // ms
    uint32_t interval_I2C = 1000;

//  Display
    absolute_time_t previous_time_Display = get_absolute_time();     // ms
    uint32_t interval_Display = 1000;

//  RADIO_TX
    absolute_time_t previous_time_RADIO_TX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_TX = 1000;

//  RADIO_RX
    absolute_time_t previous_time_RADIO_RX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_RX = 1000;

//  UART
    absolute_time_t previous_time_UART = get_absolute_time();     // ms     
    uint32_t interval_UART = 1000;

// UART2
    absolute_time_t previous_time_UART2 = get_absolute_time();     // ms
    uint32_t interval_UART2 = 1000;

// Power
    absolute_time_t previous_time_Power = get_absolute_time();     // ms
    uint32_t interval_Power = 1000;

// WDT
    absolute_time_t previous_time_WDT = get_absolute_time();     // ms      
    uint32_t interval_WDT = 1000;

// Prevent loop from burning too much CPU   
    const int LOOP_THROTTLE_DELAY_MS = 100;

    while (true) {

    // Time to LED ON?
        if (absolute_time_diff_us(previous_time_LED_ON, get_absolute_time()) >= interval_LED_ON) {
            // Save the last time you blinked the LED
            previous_time_LED_ON = get_absolute_time();    
            pico_set_led(true);
            previous_time_LED_OFF = get_absolute_time();   //Tell LED_OFF to start counting
        }
    
    // Time to LED OFF?
        if (absolute_time_diff_us(previous_time_LED_OFF, get_absolute_time()) >= interval_LED_OFF) {
            // Save the last time you blinked the LED
            previous_time_LED_OFF = get_absolute_time();    
            pico_set_led(false);
        }

    // Time to I2C Scan?    
        if (absolute_time_diff_us(previous_time_I2C, get_absolute_time()) >= interval_I2C) {
            previous_time_I2C = get_absolute_time();    
            printf("I2C Bus Scan\n");
            printf("   0  1  2  3 4 5 6 7 8 9 A B C D E F\n");

            int ret;
            uint8_t rxdata;
            // Loop over addresses and see if there's an ACK.  This is fast, otherwise it 
            // would be better to do this a frew addresses at a time.
            for (int addr = 0; addr < (1 << 7); ++addr) {
                if (addr % 16 == 0) {
                    printf("%02x ", addr);
                }

                
                if (reserved_addr(addr))
                    ret = PICO_ERROR_GENERIC;
                else
                    ret = i2c_read_blocking_until(i2c0, addr, &rxdata, 1, false, make_timeout_time_ms(100));

                printf(ret < 0 ? "." : "@");
                printf(addr % 16 == 15 ? "\n" : "  ");
            }
        }   

        // Time to Display? 
        if (absolute_time_diff_us(previous_time_Display, get_absolute_time()) >= interval_Display) {
            // Save the last time you updated the Display
            previous_time_Display = get_absolute_time();    
            printf("Display\n");
        }

        // Time to RADIO_TX?
        if (absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
            // Save the last time you TX'd on the RADIO
            previous_time_RADIO_TX = get_absolute_time();    
            printf("RADIO_TX\n");
        }

        // Time to RADIO_RX?    
        if (absolute_time_diff_us(previous_time_RADIO_RX, get_absolute_time()) >= interval_RADIO_RX) {
            // Save the last time you listened on the RADIO
            previous_time_RADIO_RX = get_absolute_time();    
            printf("RADIO_RX\n");
        }

        // Time to UART?
        if (absolute_time_diff_us(previous_time_UART, get_absolute_time()) >= interval_UART) {
            // Save the last time you blinked checked UART loopback
            previous_time_UART = get_absolute_time();    
            printf("UART\n");
        }   

        // Time to UART2?
        if (absolute_time_diff_us(previous_time_UART2, get_absolute_time()) >= interval_UART2) {
            // Save the last time you blinked checked UART2 loopback
            previous_time_UART2 = get_absolute_time();    
            printf("UART2\n");
        }

        // Time to Power?
        if (absolute_time_diff_us(previous_time_Power, get_absolute_time()) >= interval_Power) {
            // Save the last time you checked the power monitor
            previous_time_Power = get_absolute_time();    
            printf("Power\n");
        }

        // Time to WDT? 
        if (absolute_time_diff_us(previous_time_WDT, get_absolute_time()) >= interval_WDT) {
            // Save the last time you fed the Watchdog
            previous_time_WDT = get_absolute_time();    
            printf("WDT\n");
        }

        sleep_ms(LOOP_THROTTLE_DELAY_MS);

    }  // End SuperLoop
    return 0;
}   
