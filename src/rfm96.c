#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pins.h"

/*******************************************************************************
 * Function Declarations
 */
void reg_write( spi_inst_t *spi, 
    spi_pins_t spi_pins, 
    const uint8_t reg, 
    const uint8_t data);

int reg_read(  spi_inst_t *spi,
    spi_pins_t spi_pins,
    const uint8_t reg,
    uint8_t *buf,
    uint8_t nbytes);

void rfm96_reset(spi_pins_t spi_pins);

int rfm96_init(
spi_pins_t spi_pins);

/*******************************************************************************
* Function Definitions
*/

// Write 1 byte to the specified register
void reg_write( spi_inst_t *spi, 
    spi_pins_t spi_pins,
    const uint8_t reg, 
    const uint8_t data) {

uint8_t msg[2];
    
// Construct message (set ~W bit low, MB bit low)
msg[0] = 0x00 | reg;
msg[1] = data;

// Write to register
gpio_put(spi_pins.CS, 0);
spi_write_blocking(spi, msg, 2);
gpio_put(spi_pins.CS, 1);
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  spi_inst_t *spi,
    spi_pins_t spi_pins,
    const uint8_t reg,
    uint8_t *buf,
    const uint8_t nbytes) {

int num_bytes_read = 0;
uint8_t mb = 0;

// Determine if multiple byte (MB) bit should be set
if (nbytes < 1) {
return -1;
} else if (nbytes == 1) {
mb = 0;
} else {
mb = 1;
}

// Construct message (set ~W bit high)
uint8_t msg = 0x80 | (mb << 6) | reg;

// Read from register
gpio_put(spi_pins.CS, 0);
spi_write_blocking(spi, &msg, 1);
num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
gpio_put(spi_pins.CS, 1);

return num_bytes_read;
}

void rfm96_reset(spi_pins_t spi_pins)
{
    // Reset the chip as per RFM9X.pdf 7.2.2 p109
    gpio_init(spi_pins.RESET);
    gpio_set_dir(spi_pins.RESET, GPIO_OUT);
    gpio_put(spi_pins.RESET, 0);

    sleep_us(100);

    // set reset pin to input
    gpio_set_dir(spi_pins.RESET, GPIO_IN);

    sleep_ms(5);
}

int rfm96_init(spi_pins_t spi_pins) {


    // Setup cs line
    gpio_init(spi_pins.CS);
    gpio_set_dir(spi_pins.CS, GPIO_OUT);
    gpio_disable_pulls(spi_pins.CS);
    gpio_put(spi_pins.CS, 1);

    // SPI
    gpio_set_function(spi_pins.SCK, GPIO_FUNC_SPI);
    gpio_set_function(spi_pins.COPI, GPIO_FUNC_SPI);
    gpio_set_function(spi_pins.CIPO, GPIO_FUNC_SPI);

    rfm96_reset(spi_pins);


    busy_wait_ms(10);  //?

    // 
    // Ports  Seems like it should be SPI1 but flight code has SPI0
    spi_inst_t *spi = spi1;

    // Initialize SPI port at 1 MHz
    spi_init(spi, 1000*1000);

    // Set SPI bus details --RFM9X.pdf 4.3 p75: CPOL = 0, CPHA = 0 (mode 0) MSB first
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // 0x42 is the Chip ID and the value returned should be 0x11
    uint8_t v = 0;
    printf((reg_read(spi, spi_pins, 0x42, &v, 1) == 0) ? "RFM9X Chip ID read success\n" : "RFM9X Chip ID read failed\n");
    printf((v == 0x11) ? "RFM9X version check success\n" : "RFM9X version check failed\n");
    return v == 0x11;       //Allows the superloop to keep trying if it didn't initialize 
}