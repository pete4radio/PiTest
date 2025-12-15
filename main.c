

/**
 *  Pi test provides an easy to build, easy to modify bring up software for the avionics team
 * to verify hardware before passing it to the software team for integration with the flight software
 * and structures for assembling the satellite.
 *
 * To build for SAMWISE, Comment out the definition for PICO in CMakeLists.txt
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
//#include "tusb.h"                     // TinyUSB header file.  PHM: Why is this commented out?

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
#include "doUHF.h"
#include "doSband.h"
#include "sband.h"
char buffer_UART[BUFLEN] = {0}; // Define the buffer
char buffer_GPS[BUFLEN] = {0}; // Define the buffer for GPS data

// Pico W devices use a GPIO on the WIFI chip for the LED,  //PHM remove
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

//#ifndef PICO
// Ensure that PICO_RP2350A is undefined for PiCubed builds.
// pete4radio/boards/SAMWISE.h undefine it.
// The CMakeLists.txt file points to this file for the board definition.
// Check the pin is compatible with the platform
//#if 44 >= NUM_BANK0_GPIOS
//   #error "Recompile specifying the RP2350B platform SAMWISE"
//    #endif
//#endif

//PHM these should be in pins.h but it appears they are unused
//#define PIN_SDA 4
//#define PIN_SCL 5
//#define SAMWISE_BURNWIRE_PIN 6

char command_buffer[BUFLEN] = "";
char response_buffer[BUFLEN] = "";
size_t cmd_idx = 0;  // index into command_buffer
int rfm96_init(spi_pins_t *spi_pins);   //declaration for init which lives in rfm96.c
//  define storage and load them with values from pins.h
spi_pins_t spi_pins =
{
    .RESET = SAMWISE_RF_RST_PIN,    // UHF reset pin
    .CIPO = SAMWISE_RF_MISO_PIN,    // UHF MISO on GPIO 16
    .COPI = SAMWISE_RF_MOSI_PIN,    // UHF MOSI on GPIO 19
    .SCK = SAMWISE_RF_SCK_PIN,      // UHF SCK on GPIO 18
    .CS = SAMWISE_RF_CS_PIN,        // UHF chip select on GPIO 17
    .D0 = SAMWISE_RF_D0_PIN,        // UHF interrupt on GPIO 20
    .spi = spi0                     // UHF uses SPI0
};

// SBand SPI pins (separate SPI1 bus)
spi_pins_t spi_pins_sband = {
    .RESET = SAMWISE_SBAND_RST_PIN,  // SBand reset on GPIO 10
    .CIPO = SAMWISE_SBAND_MISO_PIN,  // SBand MISO on GPIO 28
    .COPI = SAMWISE_SBAND_MOSI_PIN,  // SBand MOSI on GPIO 27
    .SCK = SAMWISE_SBAND_SCK_PIN,    // SBand SCK on GPIO 26
    .CS = SAMWISE_SBAND_CS_PIN,      // SBand CS on GPIO 14
    .D0 = SAMWISE_SBAND_D0_PIN,      // SBand interrupt on GPIO 15
    .spi = spi1                       // Separate SPI1 instance
};

// for the I2C scanner, these I2C addresses are reserved
bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Perform initialisation of the tiny LED on a PICO
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
    gpio_init(SAMWISE_BURNWIRE_PIN);
    gpio_set_dir(SAMWISE_BURNWIRE_PIN, GPIO_OUT);
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

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    rc = pico_I2C_init();
    hard_assert(rc == PICO_OK);

    int ws2812rc = ws2812_init();
    white();  //  Indicate we are idle

    //Initialize serial port(s) chosen in CMakeLists.txt
    stdio_init_all();
    //  see https://forums.raspberrypi.com/viewtopic.php?t=300136
    printf("main: waiting for USB to connect\n");
    int i = 100;
    while (!stdio_usb_connected() && i--) { sleep_ms(100);  }
    printf("USB_connected or timed out\n");

//  Initialize the timing and storage variables for each test in the superloop

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

//  RADIO_TX and RX - now handled by doUHF module
    char buffer_RADIO_TX[BUFLEN] = "";      //PHM why clear the buffers?
    char buffer_RADIO_RX[BUFLEN*2] = "";
    // SBand buffers
    char buffer_Sband_RX[BUFLEN*2] = "";
    char buffer_Sband_TX[BUFLEN*2] = "";

    // Initialize UHF radio (includes ISR setup and tx_done test)
    initUHF(&spi_pins);

    // Initialize SBand radio
    printf("Initializing SBand radio...\n");
    initSband(&spi_pins_sband);

//  ws2812 LED State Management.  Neopixel LED is white while listening,
//  green while packets are in the received queue and red during transmit plus a bit extra
//  for visibility
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

    // tx_done test is now handled in initUHF()

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
            // Print SBand buffers
            // Display SBand BUSY timeout errors
            if (sband_busy_timeout_count > 0) {
                printf("SBand: BUSY timeout errors: %lu\n", sband_busy_timeout_count);
                sband_busy_timeout_count = 0;  // Clear counter after displaying
            }
            if (strlen(buffer_Sband_TX) > 0) {
                printf("SBand TX: %s", buffer_Sband_TX);
                buffer_Sband_TX[0] = '\0';  // Clear buffer
            }
            if (strlen(buffer_Sband_RX) > 0) {
                printf("SBand RX: %s\n", buffer_Sband_RX);
                buffer_Sband_RX[0] = '\0';  // Clear buffer
            }
            printf(buffer_UART);
            printf(buffer_UART2);
            printf(buffer_GPS);
            printf(buffer_WDT);
        }

// RADIO_RX and RADIO_TX: Now handled by doUHF module
        doUHF(buffer_RADIO_RX, buffer_RADIO_TX);

        // SBand operations
        doSband(buffer_Sband_RX, buffer_Sband_TX);

        // Update LED with additive color mixing from both radios
        uint8_t combined_r = uhf_led_r + sband_led_r;
        uint8_t combined_g = uhf_led_g + sband_led_g;
        uint8_t combined_b = uhf_led_b + sband_led_b;
        // Cap at 0xFF to prevent overflow
        if (combined_r > 0xFF) combined_r = 0xFF;
        if (combined_g > 0xFF) combined_g = 0xFF;
        if (combined_b > 0xFF) combined_b = 0xFF;
        set_led_color(combined_r, combined_g, combined_b);

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
            gpio_put(SAMWISE_BURNWIRE_PIN, 0);
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
            if (cmd_idx > 0 && temp == PICO_ERROR_TIMEOUT) {  // We've got a command, which come in bursts
                command_buffer[cmd_idx] = '\0';               // Null terminate the command buffer
                cmd_idx = 0;                                   // Reset the command index    
            // Process the command
                if (strcmp(command_buffer, "BURN_ON") == 0) {
                    burn_state = 1;
                    sprintf(response_buffer, "BURN_WIRE was turned on\n");
            // set gpio to burn wire to 1
                    gpio_put(SAMWISE_BURNWIRE_PIN, 1);
                    previous_time_BURN_WIRE = get_absolute_time(); 
                }
                if (strcmp(command_buffer, "RESTART_HISTOGRAM\n") == 0) { //PHM
                    // Histogram is now managed in doUHF.c module
                    printf("RESTART_HISTOGRAM command not available in refactored code\n");
                }   
            }
            else if (temp != PICO_ERROR_TIMEOUT)
            //  add character to command buffer
                command_buffer[cmd_idx++] = temp;   
            sprintf(buffer_COMMANDS, "COMMANDS last command:%s; response:%s\n", command_buffer, response_buffer);
        }

    }  // End SuperLoop
    return 0;
}   
