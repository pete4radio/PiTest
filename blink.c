

/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "pins.h"
//#include "tusb.h"

// from logger.h
#include "pico/printf.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "string.h"

// Neopixel aka WS2812
#include "ws2812.h"

//MPPT charge controllers
#include "mppt.h"

//ADM1176 power monitor
#include "power.h"

//ADS7830 ADC
#include "ads7830.h"

#include "uart.h"
#include "gps.h"
#include "main_gps_uart_shared_buffer.h"
char buffer_UART[BUFLEN] = {0}; // Define the buffer
char buffer_GPS[BUFLEN] = {0}; // Define the buffer for GPS data

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

//#ifndef PICO
// Ensure that PICO_RP2350A is undefined PICUBED builds.
// boards/samwise_picubed.h or P3_6b.h should undefine it.
// The CMakeLists.txt file points to this file for the board definition.
//Check the pin is compatible with the platform
//#if 44 >= NUM_BANK0_GPIOS
//   #error "Recompile specifying the RP2350B platform SAMWISE"
//    #endif
//#endif


#define PIN_SDA 4
#define PIN_SCL 5
#define PIN_BURN_WIRE 6

char command_buffer[BUFLEN] = "";
char response_buffer[BUFLEN] = "";
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
    gpio_set_function(SAMWISE_MPPT_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_MPPT_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_MPPT_SDA_PIN);
    gpio_pull_up(SAMWISE_MPPT_SCL_PIN);
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(SAMWISE_MPPT_SDA_PIN - 2, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_MPPT_SCL_PIN - 2, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_MPPT_SDA_PIN - 2);
    gpio_pull_up(SAMWISE_MPPT_SCL_PIN - 2);
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
    while (!stdio_usb_connected() && i--) { sleep_ms(100);  }
    printf("USB_connected or timed out\n");

//  Initialize the variables for each test 

//  LED ON
    absolute_time_t previous_time_LED_ON = get_absolute_time();     // ms
    uint32_t interval_LED_ON = 7000000;
    char buffer_LED_ON[BUFLEN] = "";

// LED OFF
    absolute_time_t previous_time_LED_OFF = get_absolute_time();     // ms
    uint32_t interval_LED_OFF = 500000;
    char buffer_LED_OFF[BUFLEN] = "";
 
//I2C Scan
    absolute_time_t previous_time_I2C = get_absolute_time();     // ms
    uint32_t interval_I2C = 1*1000*1000;  
    char buffer_I2C0[BUFLEN] = "";
    char buffer_I2C1[BUFLEN] = "";

//  Display
    absolute_time_t previous_time_Display = get_absolute_time();     // ms
    uint32_t interval_Display = 1010000;        // insure 1s period reports are fresh
    char buffer_Display[BUFLEN] = "";

//  RADIO_TX
    absolute_time_t previous_time_RADIO_TX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_TX = 2*1010* 1000;                        // Give the radio time to RX
    char buffer_RADIO_TX[BUFLEN] = "";
    radio_initialized = rfm96_init(&spi_pins);

//  RADIO_RX
    absolute_time_t previous_time_RADIO_RX = get_absolute_time();     // ms        
    uint32_t interval_RADIO_RX = 1000000;
    char buffer_RADIO_RX[BUFLEN*2] = "";
    char packet[256];  // room for incoming packet and dummy byte
    uint8_t nCRC = 0; // CRC error count

//  UART
    absolute_time_t previous_time_UART = get_absolute_time();     // ms     
    uint32_t interval_UART = 1000000;
    char buffer_UART[BUFLEN];
    buffer_UART[0] = 0x00; //  Initialize the buffer to empty

// UART2
    absolute_time_t previous_time_UART2 = get_absolute_time();     // ms
    uint32_t interval_UART2 = 1000000;
    char buffer_UART2[BUFLEN] = "";

// GPS
    absolute_time_t previous_time_GPS = get_absolute_time();     // ms
    uint32_t interval_GPS = 1000000;
    extern char buffer_GPS[BUFLEN];
    buffer_GPS[0] = 0x00; //  Initialize the buffer to empty
    gps_data_t *gps_data = malloc(sizeof(gps_data_t));
    if (!gps_data) {
        printf("ERROR: Failed to allocate memory for gps_data\n");
    }


// MPPT1
    absolute_time_t previous_time_MPPT1 = get_absolute_time();     // ms
    uint32_t interval_MPPT1 = 1000000;
    char buffer_MPPT1[BUFLEN] = "";

// MPPT2
    absolute_time_t previous_time_MPPT2 = get_absolute_time();     // ms
    uint32_t interval_MPPT2 = 1000000;
    char buffer_MPPT2[BUFLEN] = "";

// Power
    absolute_time_t previous_time_Power = get_absolute_time();     // ms
    uint32_t interval_Power = 1000000;
    char buffer_Power[BUFLEN] = "";
    float voltage, current;

//ADS7830 ADC
    uint8_t adc_voltage = 0;
    absolute_time_t previous_time_ADC = get_absolute_time();     // ms
    uint32_t interval_ADC = 1000000;
    char buffer_ADC[BUFLEN] = "";

// BURN_WIRE
    absolute_time_t previous_time_BURN_WIRE = get_absolute_time();     // ms
    uint32_t interval_BURN_WIRE = 1000000;
    char buffer_BURN_WIRE[BUFLEN] = "";

// WDT
    absolute_time_t previous_time_WDT = get_absolute_time();     // ms      
    uint32_t interval_WDT = 1000000;
    char buffer_WDT[BUFLEN] = "";

//  COMMANDS
    absolute_time_t previous_time_COMMANDS = get_absolute_time();     // ms
    uint32_t interval_COMMANDS = 1000000;
    char buffer_COMMANDS[BUFLEN] = "";


    if (radio_initialized == 0) { //check each time so radio can be hot swapped in.
        radio_initialized = rfm96_init(&spi_pins); }
    if (radio_initialized) {
//  Check that we can know here when the transmitter has completed
//  by sending a short and a long packet and measuring the time it takes
//  starting time
        red();  //  Indicate we are transmitting
        absolute_time_t start_time = get_absolute_time();
//  Send a short packet                    
        rfm96_packet_to_fifo(buffer_RADIO_TX, 5);
        rfm96_transmit();  //  Send the packet
        int ip = 10000;
        while (!rfm96_tx_done() && ip--) { sleep_us(10);  }
        if (!rfm96_tx_done()) printf("main: TX timed out\n");
        printf("Time to send a short packet: %lld ms (%d iterations left)\n", absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);
        start_time = get_absolute_time();
//  Send a long packet
        rfm96_packet_to_fifo(buffer_RADIO_TX, 250);
        rfm96_transmit();  //  Send the packet
        ip = 100000;
        while (!rfm96_tx_done() && ip--) { sleep_us(10);  }
        if (!rfm96_tx_done()) printf("main: TX timed out\n");
        printf("Time to send a long packet: %lld ms (%d iterations left)\n", absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);
        rfm96_listen(); //  Set the radio to RX mode
        green();    //  Indicate we are receiving
    }

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
            sprintf(buffer_I2C0, "I2C0: \n");
            int ret;
            uint8_t rxdata;
            for (int addr = 0; addr < (1 << 7); ++addr) {
                if (!reserved_addr(addr)) {
                    ret = i2c_read_blocking_until(i2c0, addr, &rxdata, 1, false, make_timeout_time_ms(10));
                    if (ret >= PICO_OK) { sprintf(buffer_I2C0 + strlen(buffer_I2C0) - 1, "0x%02x; \n", addr); }
                    }
            }
      
            sprintf(buffer_I2C1, "I2C1: \n");
            for (int addr = 0; addr < (1 << 7); ++addr) {
                if (!reserved_addr(addr)) {
                    ret = i2c_read_blocking_until(i2c1, addr, &rxdata, 1, false, make_timeout_time_ms(10));
                        if (ret >= PICO_OK) { sprintf(buffer_I2C1 + strlen(buffer_I2C1) - 1, "0x%02x; \n", addr); }
                    }
            }   
    }

        // Time to Display? 
        if (absolute_time_diff_us(previous_time_Display, get_absolute_time()) >= interval_Display) {
            // Save the last time you updated the Display
            previous_time_Display = get_absolute_time();    
            sprintf(buffer_Display, "Display\n");
            
            printf(buffer_BURN_WIRE);
            printf(buffer_COMMANDS);
            printf(buffer_I2C0);
            printf(buffer_I2C1);
            printf(buffer_LED_OFF);
            printf(buffer_LED_ON);
            printf(buffer_MPPT1);
            printf(buffer_MPPT2);
            printf(buffer_Power);
            printf(buffer_ADC);
            printf(buffer_RADIO_TX);
            printf("%s\n", buffer_RADIO_RX);
            printf(buffer_UART);
            printf(buffer_UART2);
            printf(buffer_GPS);
            printf(buffer_WDT);
        }

// It's always time to check for received packets. (RADIO_RX)    
            sprintf(buffer_RADIO_RX, "RXd ");     //DMA, Non-Blocking; clears out the results buffer
            if (radio_initialized) {
// if we got a packet, add it to the histogram
                while (rfm96_rx_done()) { // If this is a TX burst, keep receiving them.
                    if (rfm96_crc_error()) {
                        int remaining_space = BUFLEN - strlen(buffer_RADIO_RX) - 1; // Calculate remaining space in the buffer
                        strncat(buffer_RADIO_RX, " CRC error", remaining_space); // Safely append a newline
                        break;  //  CRC error, so ignore it
                    }
// bring in the packet from the fifo
                    packet[rfm96_packet_from_fifo(packet)] = 0; //  Terminate with a null
// how else to clear interrupts?
                    rfm96_listen(); //  Set the radio to RX mode
                    green();  //  Indicate we are receiving
// read the TX power
                    if (sscanf(packet +  15, "%d", &power) == 1) {  // Parse the integer from the packet
                        if (power >= 0 && power < 20) {       // Ensure power is within valid range
                            power_histogram[power]++;
                        } else {
                            printf("Warning: Received out-of-range power value: %d\n", power);
                        }
                    } else {
                        printf("Warning: Failed to parse power value from packet: %s.\n", packet + 4);
                        break;
                    }
                }
// Print the power histogram into buffer_RADIO_RX
                    int remaining_space = BUFLEN*2 - strlen(buffer_RADIO_RX) - 1; // Calculate remaining space in the buffer
                    for (int i = 0; (i < 20) && (remaining_space > 0); i++) {
                        int written = snprintf(buffer_RADIO_RX + strlen(buffer_RADIO_RX), remaining_space, "%d ", power_histogram[i]);
                        if (written < 0 || written >= remaining_space) {
                            printf("Warning: Buffer overflow while writing histogram\n");
                            break;
                        }
                        remaining_space -= written; // Update the remaining space
                    }               
             //  Next packet comes in at the bottom of the fifo
             rfm96_listen(); //  Set the radio to RX mode
             green();  //  Indicate we are receiving
            }

//
        sprintf(buffer_RADIO_TX, "RADIO_TXd\n");  //DMA, Non-Blocking
// Time to RADIO_TX?
         if (absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
// Save the last time you (tried to) TX on the RADIO
            previous_time_RADIO_TX = get_absolute_time();    
            if (radio_initialized == 0) { //check each time so radio can be hot swapped in.
                radio_initialized = rfm96_init(&spi_pins); }
            if (radio_initialized) {
// For range test, loop through every power level (start at 20 for actual test)
                for (int i = 10; i > 0; i--) {  // Strongest packet first, so we open the RX burst window
                    rfm96_set_tx_power(i);
// send the power level that was used
                    sprintf(buffer_RADIO_TX, "\xFF\xFF\xFF\xFFTX Power = %d", i);
                    rfm96_packet_to_fifo(buffer_RADIO_TX, strlen(buffer_RADIO_TX));
                    rfm96_transmit();  //  Send the packet
                    red();  //  Indicate we are transmitting
                    sleep_ms(230);  //  give the radio time to TX before "are we there yet?"

                    int i = 100000;
                    while (!rfm96_tx_done() && i--) { sleep_us(10);  }
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
        //  First byte of buffer is zero until the interrupt routine provides a complete line
            sprintf(buffer_UART, "");
            if (init_uart() == PICO_OK) {
                //  If we got a complete line, print it
                if (buffer_UART[0] != '\0') {
                    sprintf("UART: %s", buffer_UART);
                }
            } else {
                sprintf(buffer_UART, "UART: not found\n");
            }
        }   

        // Time to UART2?
        if (absolute_time_diff_us(previous_time_UART2, get_absolute_time()) >= interval_UART2) {
            // Save the last time you blinked checked UART2 loopback
            previous_time_UART2 = get_absolute_time();    
            sprintf(buffer_UART2, "UART2\n");
        }

        // Time to GPS?
        if (absolute_time_diff_us(previous_time_GPS, get_absolute_time()) >= interval_GPS) {
            // Save the last time you blinked checked GPS
            previous_time_GPS = get_absolute_time();
            //  Is there a line for us to decode?
            if (buffer_UART[0] != '\0'  && (gps_data != NULL)) {
                //  Decode the GPS data from the UART buffer
                do_gps(buffer_UART, gps_data);
                buffer_UART[0] = '\0'; // Clear the UART buffer after processing to accept another GPS sentence
                //  Print the GPS data  
                if (gps_data->has_fix) {
                    sprintf(buffer_GPS, "GPS Fix: %d, Lat: %.6f %c, Lon: %.6f %c, Alt: %.2f m, Speed: %.2f knots\n",
                            gps_data->fix_quality,
                            gps_data->latitude, gps_data->lat_dir,
                            gps_data->longitude, gps_data->lon_dir,
                            gps_data->altitude_m,
                            gps_data->speed_knots);
                } else {
                    sprintf(buffer_GPS, "GPS No Fix\n");    
                }
            } else {
                sprintf(buffer_GPS, "GPS\n");
            }
        }

        // Time to MPPT1?
        if (absolute_time_diff_us(previous_time_MPPT1, get_absolute_time()) >= interval_MPPT1) {
            // Save the last time you checked MPPT1
            previous_time_MPPT1 = get_absolute_time();
            sprintf(buffer_MPPT1, "MPPT1 :\n");
            if (rc=init_mppt() == 0) { do_mppt(); } //  check if the mppt is initialized
            else { sprintf(buffer_MPPT1, "MPPT1 not found\n");}
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
            if (rc=init_power(i2c0) != PICO_ERROR_GENERIC) {
                do_power(i2c0, &voltage, &current); 
                sprintf(buffer_Power + strlen(buffer_Power) - 1, " Voltage: %.2fV, Current: %.2fA\n", voltage, current);
            } 
            else {
                sprintf(buffer_Power + strlen(buffer_Power) -1, " monitor not found\n");
            }   
        }

        // Time to ADC?
        if (absolute_time_diff_us(previous_time_ADC, get_absolute_time()) >= interval_ADC) {
            // Save the last time you checked the ADC
            previous_time_ADC = get_absolute_time();    
            sprintf(buffer_ADC, "ADC \n");
            if (rc=init_ADC(i2c0) == PICO_OK) { 
                for (uint8_t channel = 0; channel < 8; channel++) { // loop through all channels
                    if (read_ADC(i2c0, channel, &adc_voltage) == PICO_OK) {
                        sprintf(buffer_ADC + strlen(buffer_ADC) - 1, "%3d; \n", adc_voltage);
                    } 
                    else {
                        sprintf(buffer_ADC + strlen(buffer_ADC) - 1, "ERR; \n");
                    }   
                }
            } 
            else {
                sprintf(buffer_ADC + strlen(buffer_ADC) -1, " not found\n");
            }   
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
            sprintf(buffer_WDT, "WDT fed\n");
            gpio_put(SAMWISE_WATCHDOG_FEED_PIN, 1);  // Set the WDT pin high
            sleep_ms(1);           // Wait for 1 ms
            gpio_put(SAMWISE_WATCHDOG_FEED_PIN, 0);  // Set the WDT pin low   
        }

        // Time to COMMANDS?
        if (absolute_time_diff_us(previous_time_COMMANDS, get_absolute_time()) >= interval_COMMANDS) {
            // Save the last time you checked for COMMANDS
            previous_time_COMMANDS = get_absolute_time();  
        // is there a character in the serial input buffer?  get it without waiting
        int temp = getchar_timeout_us(100);
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
