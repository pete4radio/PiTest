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
#include "hardware/spi.h"
#include "pins.h"
#include "tusb.h"

// from logger.h
#include "pico/printf.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "string.h"

// Neopixel aka WS2812
#include "ws2812.h"

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
int rfm96_init(spi_pins_t *spi_pins);   //declaration for init which lives in rfm96.c
//  define storage and load them with values from pins.h
spi_pins_t spi_pins =
{
    .RESET = SAMWISE_RF_RST_PIN,
    .CIPO = SAMWISE_RF_MISO_PIN,
    .COPI = SAMWISE_RF_MOSI_PIN,
    .SCK = SAMWISE_RF_SCK_PIN,
    .CS = SAMWISE_RF_CS_PIN,
    .D0 = SAMWISE_RF_D0_PIN
};

// for the I2C scanner, these I2C addresses are reserved
bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Perform initialisation of the tiny LED on a GPIO
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
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    return PICO_OK;
}

// function to turn the led on or off
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
    int burn_state = 0;             // not running the burn wire until triggered
    int radio_initialized = 0;      // radio not initialized until it is; after that we either TX or RX
    int power_histogram[20] = {0};  // histogram of received power levels
    int power = 0;                  // power level of received packet

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    rc = pico_I2C_init();
    hard_assert(rc == PICO_OK);

    rc = ws2812_init();
    white();  //  Indicate we are idle

    //Initialize serial port(s) chosen in CMakeLists.txt
    stdio_init_all();
    //  see https://forums.raspberrypi.com/viewtopic.php?t=300136
    int i = 100;
    while (!tud_cdc_connected() && i--) { sleep_ms(100);  }
    printf("USB_connected or timed out\n");

//  Initialize the variables for each test 

//  LED ON
    absolute_time_t previous_time_LED_ON = get_absolute_time();     // ms
    uint32_t interval_LED_ON = 7000000;
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
            printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

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
            printf("%s\n", buffer_RADIO_RX);
            printf(buffer_RADIO_TX);
            printf(buffer_UART);
            printf(buffer_UART2);
            printf(buffer_WDT);
        }

// It's always time to check for received packets. (RADIO_RX)    
            sprintf(buffer_RADIO_RX, "RADIO_RX");
            if (radio_initialized) {
// if we got a packet, add it to the histogram
                while (rfm96_rx_done()) { // If this is a TX burst, keep receiving them.
                    if (rfm96_crc_error()); {
                        int remaining_space = buflen - strlen(buffer_RADIO_RX) - 1; // Calculate remaining space in the buffer
                        strncat(buffer_RADIO_RX, " CRC error", remaining_space); // Safely append a newline
                        break;  //  CRC error, so ignore it
                    }
// bring in the packet from the fifo
                    char packet[256];
                    packet[rfm96_packet_from_fifo(packet)] = 0; //  Terminate with a null
// how else to clear interrupts?
                    rfm96_listen(); //  Set the radio to RX mode
                    green();  //  Indicate we are receiving
// read the TX power
                    if (sscanf(packet +  11, "%d", &power) == 1) {  // Parse the integer from the packet
                        if (power >= 0 && power < 20) {       // Ensure power is within valid range
                            power_histogram[power]++;
                        } else {
                            printf("Warning: Received out-of-range power value: %d\n", power);
                        }
                    } else {
                        printf("Warning: Failed to parse power value from packet: %s\n", packet);
                        break;
                    }
// Print the power histogram into buffer_RADIO_RX
                    int remaining_space = buflen - strlen(buffer_RADIO_RX) - 1; // Calculate remaining space in the buffer
                    for (int i = 0; i < 20 && remaining_space > 0; i++) {
                        int written = snprintf(buffer_RADIO_RX + strlen(buffer_RADIO_RX), remaining_space, "%d ", power_histogram[i]);
                        if (written < 0 || written >= remaining_space) {
                            // If snprintf fails or exceeds the remaining space, stop appending
                            break;
                        }
                        remaining_space -= written; // Update the remaining space
                    }
             }
             //  Next packet comes in at the bottom of the fifo
             rfm96_listen(); //  Set the radio to RX mode
             green();  //  Indicate we are receiving
            }

// Time to RADIO_TX?
         if (absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
// Save the last time you (tried to) TX on the RADIO
            previous_time_RADIO_TX = get_absolute_time();    
            sprintf(buffer_RADIO_TX, "RADIO_TX\n");
            if (radio_initialized == 0) { //check each time so radio can be hot swapped in.
                radio_initialized = rfm96_init(&spi_pins); }
            if (radio_initialized) {
// For range test, loop through every power level
                for (int i = 20; i > 0; i--) {  // Strongest packet first, so we open the RX burst window
                    rfm96_set_tx_power(i);
// send the power level that was used
                    sprintf(buffer_RADIO_TX, "\xFE\xFF\xFE\xFF\xFE\xFF\xFE\xFF\xFE\xFF\xFE\xFFTX Power = %d", i);
                    rfm96_packet_to_fifo(buffer_RADIO_TX, strlen(buffer_RADIO_TX));
                    rfm96_transmit();  //  Send the packet
                    red();  //  Indicate we are transmitting
                    sleep_ms(5);  //  give the radio time to TX before "are we there yet?"

                    int i = 10;
                    while (!rfm96_tx_done() && i--) { sleep_ms(100);  }
                    if (!rfm96_tx_done()) printf("main: TX timed out\n");
                }
                rfm96_listen(); //  Set the radio to RX mode
                green();    //  Indicate we are receiving
                sprintf(buffer_RADIO_TX, "RADIO_TX packets sent\n"); 
        }
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

    }  // End SuperLoop
    return 0;
}   
