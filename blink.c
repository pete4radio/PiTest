/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "tusb.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#define PIN_SDA 4
#define PIN_SCL 5
#define PIN_BURN_WIRE 6

#define buflen 60
char command_buffer[buflen] = "";
char response_buffer[buflen] = "";
int p = 0;  // pointer to the command buffer
int burn_state = 0;             // not running the burn wire until triggered
int radio_initialized = 0;      // radio not initialized until it is
int power_histogram[20] = {0};  // histogram of received power levels

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

// brurn wire GPIO initialization
int pico_burn_wire_init(void) {
    gpio_init(PIN_BURN_WIRE);
    gpio_set_dir(PIN_BURN_WIRE, GPIO_OUT);
    return PICO_OK;
}

int pico_I2C_init(void) {
    i2c_init(i2c0, 100 * 1000000);
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

    //Initialize serial port(s) chosen in CMakeLists.txt
    stdio_init_all();
    //  see https://forums.raspberrypi.com/viewtopic.php?t=300136
    int i = 100;
   while (!tud_cdc_connected() && i--) { sleep_ms(100);  }
    printf("USB_connected or timed out\n");

//  Initialize the variables for each test 

//  LED ON
    absolute_time_t previous_time_LED_ON = get_absolute_time();     // ms
    uint32_t interval_LED_ON = 1000000;
    char buffer_LED_ON[buflen] = "";

// LED OFF
    absolute_time_t previous_time_LED_OFF = get_absolute_time();     // ms
    uint32_t interval_LED_OFF = 500000;
    char buffer_LED_OFF[buflen] = "";
 
//I2C Scan
    absolute_time_t previous_time_I2C = get_absolute_time();     // ms
    uint32_t interval_I2C = 1000000;  
    char buffer_I2C[buflen] = "";

//  Display
    absolute_time_t previous_time_Display = get_absolute_time();     // ms
    uint32_t interval_Display = 1000000;
    char buffer_Display[buflen] = "";

//  RADIO_TX
    absolute_time_t previous_time_RADIO_TX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_TX = 1000000;
    char buffer_RADIO_TX[buflen] = "";
//  RADIO_RX
    absolute_time_t previous_time_RADIO_RX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_RX = 1000000;
    char buffer_RADIO_RX[buflen] = "";

//  UART
    absolute_time_t previous_time_UART = get_absolute_time();     // ms     
    uint32_t interval_UART = 1000000;
    char buffer_UART[buflen] = "";

// UART2
    absolute_time_t previous_time_UART2 = get_absolute_time();     // ms
    uint32_t interval_UART2 = 1000000;
    char buffer_UART2[buflen] = "";

// MPPT1
    absolute_time_t previous_time_MPPT1 = get_absolute_time();     // ms
    uint32_t interval_MPPT1 = 1000000;
    char buffer_MPPT1[buflen] = "";

// MPPT2
    absolute_time_t previous_time_MPPT2 = get_absolute_time();     // ms
    uint32_t interval_MPPT2 = 1000000;
    char buffer_MPPT2[buflen] = "";

// Power
    absolute_time_t previous_time_Power = get_absolute_time();     // ms
    uint32_t interval_Power = 1000000;
    char buffer_Power[buflen] = "";

// BURN_WIRE
    absolute_time_t previous_time_BURN_WIRE = get_absolute_time();     // ms
    uint32_t interval_BURN_WIRE = 1000000;
    char buffer_BURN_WIRE[buflen] = "";

// WDT
    absolute_time_t previous_time_WDT = get_absolute_time();     // ms      
    uint32_t interval_WDT = 1000000;
    char buffer_WDT[buflen] = "";

//  COMMANDS
    absolute_time_t previous_time_COMMANDS = get_absolute_time();     // ms
    uint32_t interval_COMMANDS = 1000000;
    char buffer_COMMANDS[buflen] = "";

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
                    ret = i2c_read_blocking_until(i2c0, addr, &rxdata, 1, false, make_timeout_time_ms(10));

                printf(ret < 0 ? "." : "@");
                printf(addr % 16 == 15 ? "\n" : "  ");
            }
        }   

        // Time to Display? 
        if (absolute_time_diff_us(previous_time_Display, get_absolute_time()) >= interval_Display) {
            // Save the last time you updated the Display
            previous_time_Display = get_absolute_time();    
            sprintf(buffer_Display, "Display\n");
            
            printf(buffer_BURN_WIRE);
            printf(buffer_COMMANDS);
            printf(buffer_Display);
            printf(buffer_I2C);
            printf(buffer_LED_OFF);
            printf(buffer_LED_ON);
            printf(buffer_MPPT1);
            printf(buffer_MPPT2);
            printf(buffer_Power);
            printf(buffer_RADIO_RX);
            printf(buffer_RADIO_TX);
            printf(buffer_UART);
            printf(buffer_UART2);
            printf(buffer_WDT);
        }

        // Time to RADIO_TX?
        if (absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
            // Save the last time you TX'd on the RADIO
            previous_time_RADIO_TX = get_absolute_time();    
            sprintf(buffer_RADIO_TX, "RADIO_TX\n");
            if (radio_initialized == 0) {  //check each time so radio can be hot swapped in.
                //slate->radio = rfm9x_mk(RFM9X_SPI, RFM9X_RESET, RFM9X_CS, RFM9X_TX, RFM9X_RX, RFM9X_CLK);
                //rfm9x_init(&slate->radio);
                //printf("Brought up RFM9X v%d", rfm9x_version(&slate->radio));
                radio_initialized = 1;
            }
            // For range test, loop through every power level
            //for (int i = 0; i < 20; i++) {
            //rfm9x_set_tx_power(r, i);
            // send the power level that was used
            //sprintf(buffer_RADIO_TX, "RADIO_TX power level %d\n", i);
            //rfm9x_send(rfm9x_t *r, char *buffer_RADIO_TX, uint32_t l, uint8_t keep_listening,
            //    uint8_t destination, uint8_t node, uint8_t identifier,
             //   uint8_t flags);
        }

        // Time to check for received packets? (RADIO_RX?)    
        if (absolute_time_diff_us(previous_time_RADIO_RX, get_absolute_time()) >= interval_RADIO_RX) {
            // Save the last time you listened on the RADIO
            previous_time_RADIO_RX = get_absolute_time();    
            sprintf(buffer_RADIO_RX, "RADIO_RX\n");
            if (radio_initialized == 0) {
                //slate->radio = rfm9x_mk(RFM9X_SPI, RFM9X_RESET, RFM9X_CS, RFM9X_TX, RFM9X_RX, RFM9X_CLK);
                //rfm9x_init(&slate->radio);
                //printf("Brought up RFM9X v%d", rfm9x_version(&slate->radio));
                radio_initialized = 1;
            }
            // if we got a packet, add it to the histogram
            //power_histogram[power]++;
            //rfm9x_receive(rfm9x_t *r, char *packet, uint8_t node,
            //    uint8_t keep_listening, uint8_t with_ack);
            // // Print the power histogram into buffer_RADIO_RX
            for (int i = 0; i < 20; i++) {
                char temp[10];
                sprintf(temp, "%d ", power_histogram[i]);
                strcat(buffer_RADIO_RX, temp);
            }
            strcat(buffer_RADIO_RX, "\n");
            printf("%s", buffer_RADIO_RX);
        }

        // Time to UART?
        if (absolute_time_diff_us(previous_time_UART, get_absolute_time()) >= interval_UART) {
            // Save the last time you blinked checked UART loopback
            previous_time_UART = get_absolute_time();    
            sprintf(buffer_UART, "UART\n");
        }   

        // Time to UART2?
        if (absolute_time_diff_us(previous_time_UART2, get_absolute_time()) >= interval_UART2) {
            // Save the last time you blinked checked UART2 loopback
            previous_time_UART2 = get_absolute_time();    
            sprintf(buffer_UART2, "UART2\n");
        }

        // Time to MPPT1?
        if (absolute_time_diff_us(previous_time_MPPT1, get_absolute_time()) >= interval_MPPT1) {
            // Save the last time you checked MPPT1
            previous_time_MPPT1 = get_absolute_time();    
            sprintf(buffer_MPPT1, "MPPT1\n");
        }

        // Time to MPPT2?
        if (absolute_time_diff_us(previous_time_MPPT2, get_absolute_time()) >= interval_MPPT2) {
            // Save the last time you checked MPPT2
            previous_time_MPPT2 = get_absolute_time();    
            sprintf(buffer_MPPT2, "MPPT2\n");
        }

        // Time to Power?
        if (absolute_time_diff_us(previous_time_Power, get_absolute_time()) >= interval_Power) {
            // Save the last time you checked the power monitor
            previous_time_Power = get_absolute_time();    
            sprintf(buffer_Power, "Power\n");
        }

        // time to BURN_WIRE ?
        if (absolute_time_diff_us(previous_time_BURN_WIRE, get_absolute_time()) >= interval_BURN_WIRE) {
            burn_state = 0;
            // set gpio to burn wire to 0
            gpio_put(PIN_BURN_WIRE, 0);
            sprintf(buffer_BURN_WIRE, "BURN_WIRE off\n");
        }

        // Time to WDT? 
        if (absolute_time_diff_us(previous_time_WDT, get_absolute_time()) >= interval_WDT) {
            // Save the last time you fed the Watchdog
            previous_time_WDT = get_absolute_time();    
            sprintf(buffer_WDT, "WDT\n");
        }

        // Time to COMMANDS?
        if (absolute_time_diff_us(previous_time_COMMANDS, get_absolute_time()) >= interval_COMMANDS) {
            // Save the last time you checked for COMMANDS
            previous_time_COMMANDS = get_absolute_time();  
        // is there a character in the serial input buffer?  get it without waiting
        int temp = getchar_timeout_us(0);
            if (p > 0 && temp == PICO_ERROR_TIMEOUT) {  // We've got a command, which come in bursts
                command_buffer[p++] = '\0';    // Null terminate the command buffer
                p = 0;                      // Reset the pointer    
            // Process the command
                if (strcmp(command_buffer, "BURN_ON") == 0) {
                    burn_state = 1;
                    sprintf(response_buffer, "BURN_WIRE was turned on\n");
            // set gpio to burn wire to 1
                    gpio_put(PIN_BURN_WIRE, 1);
                    previous_time_BURN_WIRE = get_absolute_time(); 
                }
                if (strcmp(command_buffer, "RESTART_HISTOGRAM\n") == 0) {
                    // Zero out the array
                    memset(power_histogram, 0, sizeof(power_histogram));
                }   
            }
            else if (temp != PICO_ERROR_TIMEOUT)
            //  add character to command buffer
                command_buffer[p++] = temp;   
            sprintf(buffer_COMMANDS, "COMMANDS last command:%s; response:%s\n", command_buffer, response_buffer);
        }

        sleep_ms(LOOP_THROTTLE_DELAY_MS);

    }  // End SuperLoop
    return 0;
}   
