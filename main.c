

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

// Packet queue structure definitions (before main for ISR access)
#define QUEUE_SIZE 15
#define PACKET_SIZE 256
typedef struct {
    uint8_t data[PACKET_SIZE];
    uint8_t length;
    int8_t snr;
    int16_t rssi;
} packet_t;

// Global queue variables for ISR access
volatile packet_t packet_queue[QUEUE_SIZE];
volatile uint8_t queue_head = 0;  // ISR writes here
volatile uint8_t queue_tail = 0;  // Main loop reads here
volatile uint8_t queue_count = 0;
volatile absolute_time_t last_packet_time;  // For LED green timing

/*
 * DIO0 GPIO Interrupt Service Routine
 * Triggered when a packet is received (RX_DONE interrupt)
 * Copies packet from radio FIFO to queue using non-blocking DMA
 */
void dio0_isr() {
    // Check if this is an RX_DONE interrupt (not TX_DONE)
    uint8_t irq_flags = rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS);

    if (irq_flags & 0x40) {  // RX_DONE bit (bit 6)
        // Check if queue has space
        if (queue_count < QUEUE_SIZE) {
            volatile packet_t *pkt = &packet_queue[queue_head];

            // Read packet from FIFO using non-blocking DMA
            pkt->length = rfm96_packet_from_fifo((uint8_t*)pkt->data);

            // Get SNR and RSSI
            pkt->snr = rfm96_get_snr();
            pkt->rssi = rfm96_get_rssi();

            // Update SNR and RSSI in packet starting at byte 25
            // Format: "SNR = %4d; RSSI = %4d"
            if (pkt->length > 25) {
                snprintf((char*)&pkt->data[25], 25, "SNR = %4d; RSSI = %4d",
                         (int)pkt->snr, (int)pkt->rssi);
            }

            // Update queue
            queue_head = (queue_head + 1) % QUEUE_SIZE;
            queue_count++;
            last_packet_time = get_absolute_time();
        }

        // Clear RX_DONE interrupt flag
        rfm96_put8(_RH_RF95_REG_12_IRQ_FLAGS, 0x40);

        // Continue listening for next packet
        rfm96_listen();
    }
    // Note: If TX_DONE fires (bit 3), we ignore it - ISR is only for RX
}

int main() {
    int counter = 0;
    int burn_state = 0;             // not running the burn wire until triggered
    int radio_initialized = 0;      // radio not initialized until it is; after that we either TX or RX

    int power_histogram[20] = {0};  // histogram of received power levels
    int power = 0;                  // power level of received packet

    // Initialize packet queue timing
    last_packet_time = get_absolute_time();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    rc = pico_I2C_init();
    hard_assert(rc == PICO_OK);

    int ws2812rc = ws2812_init();
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
    uint8_t tx_packet[250];  // Separate 250-byte buffer for TX packets
    radio_initialized = rfm96_init(&spi_pins);

    // Set up DIO0 GPIO interrupt for packet reception
    if (radio_initialized) {
        gpio_set_irq_enabled_with_callback(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true, &dio0_isr);
        printf("main: DIO0 ISR enabled on GPIO %d\n", SAMWISE_RF_D0_PIN);
    }

//  RADIO_RX
    absolute_time_t previous_time_RADIO_RX = get_absolute_time();     // ms
    uint32_t interval_RADIO_RX = 1000000;
    char buffer_RADIO_RX[BUFLEN*2] = "";
    char packet[256];  // room for incoming packet and dummy byte
    uint8_t nCRC = 0; // CRC error count

//  ws2812 LED State Management.  Used to have the LED green while packets
//  are in the received queue plus a timed check to turn it off after a delay
    absolute_time_t previous_time_LED = get_absolute_time();
    uint32_t interval_LED = 100000;  // Check LED state every 100ms
    bool led_green_active = false;

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
//  Disable ISR during test to prevent interference with tx_done()
        gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, false);

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

//  Re-enable ISR after test completes
        gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true);
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

// RADIO_RX: Process packet queue (filled by ISR)
        if (absolute_time_diff_us(previous_time_RADIO_RX, get_absolute_time()) >= interval_RADIO_RX) {
            previous_time_RADIO_RX = get_absolute_time();
            sprintf(buffer_RADIO_RX, "RXd ");     //DMA, Non-Blocking; clears out the results buffer

            if (radio_initialized && queue_count > 0) {
                // Process all packets in queue
                while (queue_count > 0) {
                    volatile packet_t *pkt = &packet_queue[queue_tail];

                    // Parse power value from packet
                    // Format: "\xFF\xFF\xFF\xFFTX Power = %02d"
                    if (sscanf((char*)pkt->data + 4, "TX Power = %d", &power) == 1) {
                        // Adjust for negative power values (range: -1 to 17)
                        int hist_index = power + 1;  // Shift so -1 maps to 0
                        if (hist_index >= 0 && hist_index < 20) {
                            power_histogram[hist_index]++;
                        } else {
                            printf("Warning: Received out-of-range power value: %d\n", power);
                        }
                    } else {
                        printf("Warning: Failed to parse power from packet\n");
                    }

                    // Update queue
                    queue_tail = (queue_tail + 1) % QUEUE_SIZE;
                    queue_count--;
                }

                // Print the power histogram into buffer_RADIO_RX
                int remaining_space = BUFLEN*2 - strlen(buffer_RADIO_RX) - 1;
                for (int i = 0; (i < 20) && (remaining_space > 0); i++) {
                    int written = snprintf(buffer_RADIO_RX + strlen(buffer_RADIO_RX), remaining_space,
                                          "%d ", power_histogram[i]);
                    if (written < 0 || written >= remaining_space) {
                        printf("Warning: Buffer overflow while writing histogram\n");
                        break;
                    }
                    remaining_space -= written;
                }
            }
        }

// RADIO_TX: Send packets at each power level from 10 down to -1
        sprintf(buffer_RADIO_TX, "RADIO_TXd\n");  //DMA, Non-Blocking
        if (absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
            // Save the last time you transmitted
            previous_time_RADIO_TX = get_absolute_time();
            if (radio_initialized == 0) { //check each time so radio can be hot swapped in.
                radio_initialized = rfm96_init(&spi_pins);
            }
            if (radio_initialized) {
                // Disable ISR during TX
                gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, false);

                // For range test, loop through every power level from 10 to -1
                // Format: "\xFF\xFF\xFF\xFFTX Power = %02d" + spaces to 250 bytes
                for (int pwr = 10; pwr >= -1; pwr--) {
                    rfm96_set_tx_power(pwr);
                    red();  //  Indicate we are transmitting

                    // Create packet: preamble + power + spaces to fill 250 bytes
                    memset(tx_packet, ' ', 250);  // Fill with spaces
                    tx_packet[0] = '\xFF';
                    tx_packet[1] = '\xFF';
                    tx_packet[2] = '\xFF';
                    tx_packet[3] = '\xFF';
                    // Format power with leading zero or minus sign
                    sprintf((char*)tx_packet + 4, "TX Power = %02d", pwr);
                    // Restore spaces after sprintf's null terminator
                    for (int j = strlen((char*)tx_packet); j < 250; j++) {
                        tx_packet[j] = ' ';
                    }

                    rfm96_packet_to_fifo(tx_packet, 250);
                    rfm96_transmit();  //  Send the packet

                    // Wait for TX completion
                    int timeout = 100000;
                    while (!rfm96_tx_done() && timeout--) { sleep_us(10); }
                    if (!rfm96_tx_done()) printf("main: TX timed out at power %d\n", pwr);

                    sleep_ms(50);  //  Inter-packet spacing
                }

                // Re-enable ISR and return to RX mode
                gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true);
                rfm96_listen();
                sprintf(buffer_RADIO_TX, "RADIO_TX packets sent (10 to -1 dBm)\n");
            }
        }

        // Time to update LED?
        if (absolute_time_diff_us(previous_time_LED, get_absolute_time()) >= interval_LED) {
            previous_time_LED = get_absolute_time();

            if (radio_initialized) {
                // LED is already red() during transmission (set in TX task)
                // Manage LED color when listening based on queue state
                if (queue_count == 0) {
                    // Queue empty, listening - white LED
                    white();
                } else {
                    // Queue has entries - check if we're within 500ms of last packet
                    uint64_t time_since_last_packet_us = absolute_time_diff_us(last_packet_time, get_absolute_time());
                    if (time_since_last_packet_us < 500000) {  // 500ms = 500000us
                        // Within 500ms of last dequeue - green LED
                        green();
                    } else {
                        // More than 500ms since last dequeue - white LED
                        white();
                    }
                }
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
