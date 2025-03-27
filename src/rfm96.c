#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pins.h"

// Strategy differs from flight software.   Check only  the radio functionality & do range test:
//   o  SPI is used everywhere, so lets just make it a global.  CS is used only in init and 
// at lowest level routines and it's clumsy to pass it around.  Use SAMWISE_RF_CS_PIN.
//   o  pass by value (except buffers) to avoid corruption and nasty bugs later (this code needs
// to be easy to use and modify).
//   o  No interrupts because when we're doing the radio, we're not doing anything else.
//   o  For our purposes, a packet is a received transmission (no src and dest)
// 
spi_inst_t *global_spi = spi1;  

#define DEBUG 0

/*******************************************************************************
 * Function Declarations
 */
void reg_write(const uint8_t reg, const uint8_t data);

int reg_read(const uint8_t reg, uint8_t *buf, uint8_t nbytes);

void rfm96_reset();

int rfm96_init(spi_pins_t *spi_pins);

/*******************************************************************************
* Function Definitions
*/

// Write 1 byte to the specified register
void reg_write(const uint8_t reg, const uint8_t data) {

    uint8_t msg[2];    // Need two bytes as we need to clock out the address and the data
        
    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x00 | reg;
    msg[1] = data;

    // CS is active low:  Write to register
    gpio_put(SAMWISE_RF_CS_PIN, 0);
    spi_write_blocking(global_spi, msg, 2);
    gpio_put(SAMWISE_RF_CS_PIN, 1);
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

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
    gpio_put(SAMWISE_RF_CS_PIN, 0);
    spi_write_blocking(global_spi, &msg, 1);
    num_bytes_read = spi_read_blocking(global_spi, 0, buf, nbytes);
    gpio_put(SAMWISE_RF_CS_PIN, 1);

    return num_bytes_read;
}

/*
 * RFM9X SPI transaction code.
 *
 * See RFM9X.pdf 4.3 p75
 *
 * One thing that isn't entirely clear from the docs is that the device expects
 * a 0x00 written for every data byte read, as in and out use the same clock
 */

void cs_select()
 {
     busy_wait_us(5);
     gpio_put(SAMWISE_RF_CS_PIN, 0);    //CS is active low
     busy_wait_us(5);
 }
 
void cs_deselect()
 {
     busy_wait_us(5);
     gpio_put(SAMWISE_RF_CS_PIN, 1);
     busy_wait_us(5);
 }
 
 /*
  * Read a buffer from a register address.
  */
void rfm96_get_buf(rfm96_reg_t reg, uint8_t *buf, uint32_t n)
 {
     cs_select();
 
     // First, configure that we will be GETTING from the Radio Module.
     uint8_t value = reg & 0x7F;
 
     // WRITES to the radio module the value, of length 1 byte, that says that we
     // are GETTING
     spi_write_blocking(global_spi, &value, 1);
 
     // GETS from the radio module the buffer.
     // The 0 represents the arbitrary byte that should be passed IN as part of
     // the SPI shared clock.
     spi_read_blocking(global_spi, 0, buf, n);
 
     cs_deselect();
 }
 
 /*
  * Write a buffer to a register address.
  */
  void rfm96_put_buf(rfm96_reg_t reg, uint8_t *buf, uint32_t n)
 {
     cs_select();
 
     // this value will be passed in to tell the radio that we will be writing data
     uint8_t value = reg | 0x80;
 
     spi_write_blocking(global_spi, &value, 1);
 
     // Write the buffer to the radio
     spi_write_blocking(global_spi, buf, n);
 
     cs_deselect();
 }
 
 /*
  * Write a single byte to an RFM9X register
  */
 void rfm96_put8(rfm96_reg_t reg, uint8_t v) { rfm96_put_buf(reg, &v, 1); }
  
 /*
  * Get a single byte from an RFM9X register
  */
uint8_t rfm96_get8(rfm96_reg_t reg)
 {
     uint8_t v = 0;
     rfm96_get_buf(reg, &v, 1);
     return v;
 }
 
 void rfm96_reset()
 {
     // Reset the chip as per RFM9X.pdf 7.2.2 p109
 
     // set reset pin to output
     gpio_set_dir(SAMWISE_RF_RST_PIN, GPIO_OUT);
     gpio_put(SAMWISE_RF_RST_PIN, 0);
 
     sleep_us(100);
 
     // set reset pin to input.  the RESET line is active low and to
     //  run the radio, it must be high Z.
     gpio_set_dir(SAMWISE_RF_RST_PIN, GPIO_IN);
 
     sleep_ms(5);
 }
 
 /*
  * Register access helpers
  */
 
 // (RFM9X 6.2 p87)
 
 /*
  * Set mode (RFM9X 6.2 p87)
  */
void rfm96_set_mode(rfm96_mode_t mode)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     reg = bits_set(reg, 0, 2, mode);
     rfm96_put8(_RH_RF95_REG_01_OP_MODE, reg);
 }
 
 /*
  * Get mode (RFM9X 6.2 p87)
  */
uint8_t rfm96_get_mode()
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     return bits_get(reg, 0, 2);
 }
 
 /*
  * Set low frequency mode (RFM9X 6.2 p87)
  */
  void rfm96_set_low_freq_mode(uint8_t low_freq)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     if (low_freq)
         reg = bit_set(reg, 3);
     else
         reg = bit_clr(reg, 3);
     rfm96_put8(_RH_RF95_REG_01_OP_MODE, reg);
 }
 
 /*
  * Get low frequency mode (RFM9X 6.2 p87)
  */
  uint8_t rfm96_get_low_freq_mode(spi_pins_t spi_pins)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     return bit_is_on(reg, 3);
 }
 
 /*
  * Set loRa mode (chirp spread spectrum)
  * (RFM9X.pdf 6.2 p87)
  */
  void rfm96_set_lora(uint8_t lora)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     if (lora)
         reg = bit_set(reg, 7);
     else
         {
            reg = bit_clr(reg, 7);
            printf("rfm96_set_lora: Warning, LoRa mode was set off\n");
         }
     rfm96_put8(_RH_RF95_REG_01_OP_MODE, reg);
 }
 
 /*
  * Get long range mode (LoRa status)
  * (RFM9X.pdf 6.2 p87)
  */
  uint8_t rfm96_get_lora(spi_pins_t spi_pins)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     return bit_is_on(reg, 7);
 }
 
 /*
  * Triggers oscillator calibration (RFM9X.pdf 6.2 p93)
  *
  * Must be done outside of LoRa mode, since register 0x24 is aliased.
  */
  void rfm96_trigger_osc_calibration()
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_24_HOP_PERIOD);
     reg = bit_set(reg, 3);
     rfm96_put8(_RH_RF95_REG_24_HOP_PERIOD, reg);
 }
 
 /*
  * Set frequency in hz (RFM9X.pdf 6.4 p102)
  */
  void rfm96_set_frequency(uint32_t f)
 {
     uint32_t frf = (f / _RH_RF95_FSTEP) & 0xFFFFFF;
     uint8_t msb = (frf >> 16) & 0xFF;
     uint8_t mid = (frf >> 8) & 0xFF;
     uint8_t lsb = frf & 0xFF;
     rfm96_put8(_RH_RF95_REG_06_FRF_MSB, msb);
     rfm96_put8(_RH_RF95_REG_07_FRF_MID, mid);
     rfm96_put8(_RH_RF95_REG_08_FRF_LSB, lsb);
     printf("rfm96: Set frequency to %d Hz\n", f);
 }
 
 /*
  * Get frequency in hz (RFM9X.pdf 6.4 p102)
  */
  uint32_t rfm96_get_frequency()
 {
     uint32_t msb = rfm96_get8(_RH_RF95_REG_06_FRF_MSB);
     uint32_t mid = rfm96_get8(_RH_RF95_REG_07_FRF_MID);
     uint32_t lsb = rfm96_get8(_RH_RF95_REG_08_FRF_LSB);
     uint32_t frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF;
     return (frf * _RH_RF95_FSTEP);
 }
 
 /*
  * Set preamble length (RFM9X.pdf 6.4 p107)
  */
 void rfm96_set_preamble_length(uint16_t l)
 {
     rfm96_put8(_RH_RF95_REG_20_PREAMBLE_MSB, l >> 8);
     rfm96_put8(_RH_RF95_REG_21_PREAMBLE_LSB, l & 0xFF);
 }
 
 /*
  * Get preamble length (RFM9X.pdf 6.4 p107)
  */
 uint16_t rfm96_get_preamble_length()
 {
     uint16_t msb = rfm96_get8(_RH_RF95_REG_20_PREAMBLE_MSB);
     uint16_t lsb = rfm96_get8(_RH_RF95_REG_21_PREAMBLE_LSB);
 
     return (msb << 8) | lsb;
 }
 
 /*
  * Set the coding rate. Takes the denominator under 4. Valid values are [5,8].
  *
  * See RFM9X.pdf 6.4 p106
  */
 void rfm96_set_coding_rate(uint8_t v)
 {
     uint8_t denominator = 5;
     if (v > 5)
         denominator = v;
     if (v > 8)
         denominator = 8;
    printf("rfm96: Set coding rate to %d\n", denominator);
 
     uint8_t cr_id = denominator - 4;
     uint8_t config = rfm96_get8(_RH_RF95_REG_1D_MODEM_CONFIG1);
     config = bits_set(config, 1, 3, cr_id);
     rfm96_put8(_RH_RF95_REG_1D_MODEM_CONFIG1, config);
 }
 
 /*
  * Get the coding rate. Returns the denominator under 4.
  *
  * See RFM9X.pdf 6.4 p106
  */
 uint8_t rfm96_get_coding_rate()
 {
     uint8_t config = rfm96_get8(_RH_RF95_REG_1D_MODEM_CONFIG1);
     return bits_get(config, 1, 3) + 4;
 }
 
 /*
  * Set the spreading factor as base-2 logarithm. Valid values are [6,12].
  *
  * See RFM9X.pdf 6.4 p107
  */
 void rfm96_set_spreading_factor(uint8_t v)
 {
     uint8_t factor = 6;
     if (v > 6)
         factor = v;
     if (v > 12)
         factor = 12;
 
     // We skip all the DETECTION_OPTIMIZE and DETECTION_THRESHOLD stuff because
     // it isn't relevant to the RFM9X family.
 
     uint8_t c = rfm96_get8(_RH_RF95_REG_1E_MODEM_CONFIG2);
     c = bits_set(c, 4, 7, factor);
     rfm96_put8(_RH_RF95_REG_1E_MODEM_CONFIG2, c);
 }
 
 /*
  * Get the spreading factor as base-2 logarithm. Returns values [6,12].
  *
  * See RFM9X.pdf 6.4 p107
  */
 uint8_t rfm96_get_spreading_factor()
 {
     return bits_get(rfm96_get8(_RH_RF95_REG_1E_MODEM_CONFIG2), 4, 7);
 }
 
 /*
  * Enable or disable CRC checking
  *
  * See RFM9X.pdf 6.4 p107
  */
 void rfm96_set_crc(uint8_t crc)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_1E_MODEM_CONFIG2);
     if (crc)
         c = bit_set(c, 2);
     else
         c = bit_clr(c, 2);
     rfm96_put8(_RH_RF95_REG_1E_MODEM_CONFIG2, c);
 }
 
 /*
  * Get CRC checking status
  *
  * See RFM9X.pdf 6.4 p107
  */
 uint8_t rfm96_is_crc_enabled()
 {
     return bit_is_on(rfm96_get8(_RH_RF95_REG_1E_MODEM_CONFIG2), 2);
 }
 
 /*
  * check if we had a CRC error
  */
 uint8_t rfm96_crc_error()
 {
     return (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x20) >> 5;
 }
 
 /*
  * Set raw output power. (RFM9X.pdf 6.4 p103)
  */
 void rfm96_set_raw_tx_power(uint8_t power)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_09_PA_CONFIG);
     c = bits_set(c, 0, 3, power);
     rfm96_put8(_RH_RF95_REG_09_PA_CONFIG, c);
 }
 
 /*
  * Get raw output power. (RFM9X.pdf 6.4 p103)
  */
 uint8_t rfm96_get_raw_tx_power()
 {
     return bits_get(rfm96_get8(_RH_RF95_REG_09_PA_CONFIG), 0, 3);
 }
 
 /*
  * Set max power. (RFM9X.pdf 6.4 p103)
  */
 void rfm96_set_max_power(uint8_t power)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_09_PA_CONFIG);
     c = bits_set(c, 4, 6, power);
     rfm96_put8(_RH_RF95_REG_09_PA_CONFIG, c);
 }
 
 /*
  * Get max power. (RFM9X.pdf 6.4 p103)
  */
 uint8_t rfm96_get_max_power()
 {
     return bits_get(rfm96_get8(_RH_RF95_REG_09_PA_CONFIG), 4, 6);
 }
 
 /*
  * Enable or disable PA output pin
  *
  * See RFM9X.pdf 6.4 p103
  */
 void rfm96_set_pa_output_pin(uint8_t select)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_09_PA_CONFIG);
     if (select)
         c = bit_set(c, 7);
     else
         c = bit_clr(c, 7);
     rfm96_put8(_RH_RF95_REG_09_PA_CONFIG, c);
 }
 
 /*
  * Get PA output pin setting
  *
  * See RFM9X.pdf 6.4 p103
  */
 uint8_t rfm96_get_pa_output_pin()
 {
     return bit_is_on(rfm96_get8(_RH_RF95_REG_09_PA_CONFIG), 7);
 }
 
 /*
  * Set PA ramp. (RFM9X.pdf 6.4 p103)
  */
 void rfm96_set_pa_ramp(uint8_t ramp)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_0A_PA_RAMP);
     c = bits_set(c, 0, 3, ramp);
     rfm96_put8(_RH_RF95_REG_0A_PA_RAMP, c);
 }
 
 /*
  * Get PA ramp. (RFM9X.pdf 6.4 p103)
  */
 uint8_t rfm96_get_pa_ramp()
 {
     return bits_get(rfm96_get8(_RH_RF95_REG_0A_PA_RAMP), 0, 3);
 }
 
 /*
  * Set PA DAC (RFM9X.pdf 6.1 p84)
  */
 void rfm96_set_pa_dac(uint8_t dac)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_4D_PA_DAC);
     c = bits_set(c, 0, 4, dac);
     rfm96_put8(_RH_RF95_REG_4D_PA_DAC, c);
 }
 
 /*
  * Get PA ramp. (RFM9X.pdf 6.4 p103)
  *
  * Note: not entirely accurate, value should be returned in its entirety. It's
  * like this to be symmetrical with rfm96_set_pa_dac
  */
 uint8_t rfm96_get_pa_dac(spi_pins_t spi_pins)
 {
     return bits_get(rfm96_get8(_RH_RF95_REG_4D_PA_DAC), 0, 4);
 }
 
 #define BW_BIN_COUNT 9
 static uint32_t bw_bins[BW_BIN_COUNT + 1] = {
     7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 0};
 
 void rfm96_set_bandwidth(uint32_t bandwidth)
 {
     uint8_t bin = 9;
     for (uint8_t i = 0; bw_bins[i] != 0; i++)
     {
         if (bandwidth <= bw_bins[i])
         {
             bin = i;
             printf("rfm96.c: Bandwidth was set to %d\n", bw_bins[i]);
             break;
         }
     }
 
     uint8_t c = rfm96_get8(_RH_RF95_REG_1D_MODEM_CONFIG1);
     c = bits_set(c, 4, 7, bin);
     rfm96_put8(_RH_RF95_REG_1D_MODEM_CONFIG1, c);
 
     if (bandwidth >= 500000)
     {
         /* see Semtech SX1276 errata note 2.1 */
         rfm96_put8(0x36, 0x02);
         rfm96_put8(0x3a, 0x64);
     }
     else
     {
         if (bandwidth == 7800)
         {
             rfm96_put8(0x2F, 0x48);
         }
         else if (bandwidth >= 62500)
         {
             /* see Semtech SX1276 errata note 2.3 */
             rfm96_put8(0x2F, 0x40);
         }
         else
         {
             rfm96_put8(0x2F, 0x44);
         }
 
         rfm96_put8(0x30, 0);
     }
 }
 
 uint32_t rfm96_get_bandwidth()
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_1D_MODEM_CONFIG1);
     c = bits_get(c, 4, 7);
 
     if (c >= BW_BIN_COUNT)
         return 500000;
     else
         return bw_bins[c];
 }
 
 /*
  * Set the TX power. If chip is high power, valid values are [5, 23], 
  * otherwise [-1, 14]
  */
 void rfm96_set_tx_power(int8_t power)
 {
     if (0)     // max power out of reach when laptop-powered
     {
         rfm96_put8(_RH_RF95_REG_0B_OCP, 0x3F); /* set ocp to 240mA */
         rfm96_set_pa_dac(_RH_RF95_PA_DAC_ENABLE);
         rfm96_set_pa_output_pin(1);
         rfm96_set_max_power(0b111);
         rfm96_set_tx_power(0x0F);
         return;
     }
 
     if (0)    // high power out of reach when laptop-powered
     {
         if (power > 23)
             power = 23;
         if (power < 5)
             power = 5;
 
         if (power > 20)
         {
             rfm96_set_pa_dac(_RH_RF95_PA_DAC_ENABLE);
             power -= 3;
         }
         else
         {
             rfm96_set_pa_dac(_RH_RF95_PA_DAC_DISABLE);
         }
         rfm96_set_pa_output_pin(1);
         rfm96_set_tx_power((power - 5) & 0xF);
     }
     else
     {
         if (power > 14)
             power = 14;
         if (power < -1)
             power = -1;
 
         rfm96_set_pa_output_pin(1);
         rfm96_set_max_power(0b111);
         rfm96_set_raw_tx_power((power + 1) & 0x0F);
     }
 }
 
 /*
  * Get the TX power. If chip is high power, valid values are [5, 23], otherwise
  * [-1, 14]
  */
 int8_t rfm96_get_tx_power()
 {
     if (0) // high power out of reach when laptop-powered
     {
         return rfm96_get_raw_tx_power() + 5;
     }
     else
     {
         return (int8_t)rfm96_get_raw_tx_power() - 1;
     }
 }
 
 void rfm96_set_lna_boost(uint8_t boost)
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_0C_LNA);
     c = bits_set(c, 0, 1, boost);
     rfm96_put8(_RH_RF95_REG_0C_LNA, c);
 }
 
 uint8_t rfm96_get_lna_boost()
 {
     uint8_t c = rfm96_get8(_RH_RF95_REG_0C_LNA);
     c = bits_get(c, 0, 1);
     return c;
 }
 
 
 int rfm96_init(spi_pins_t *spi_pins)
 //  Initialization sets up pins, does a reset, allocates and checks the chip ID
//  The argument is passed by reference so spi can be modified
    { 
 #ifndef PICO
     // Setup RF regulator on the PiCubed board
     gpio_init(SAMWISE_RF_REGULATOR_PIN);
     gpio_set_dir(SAMWISE_RF_REGULATOR_PIN, GPIO_OUT);
 
 #ifdef BRINGUP
     gpio_put(AMWISE_RF_REGULATOR_PIN, 0);
 #else
     gpio_put(SAMWISE_RF_REGULATOR_PIN, 1);
 #endif
 
 #endif
 
     // Setup reset line
     gpio_init(SAMWISE_RF_RST_PIN);
     gpio_set_dir(SAMWISE_RF_RST_PIN, GPIO_IN);
     gpio_disable_pulls(SAMWISE_RF_RST_PIN);
     gpio_put(SAMWISE_RF_RST_PIN, 1);
 
     // Setup cs line
     gpio_init(SAMWISE_RF_CS_PIN);
     gpio_set_dir(SAMWISE_RF_CS_PIN, GPIO_OUT);
     gpio_disable_pulls(SAMWISE_RF_CS_PIN);
     gpio_put(SAMWISE_RF_CS_PIN, 1);
 
     // SPI
     gpio_set_function(SAMWISE_RF_SCK_PIN, GPIO_FUNC_SPI);
     gpio_set_function(SAMWISE_RF_MOSI_PIN, GPIO_FUNC_SPI);
     gpio_set_function(SAMWISE_RF_MISO_PIN, GPIO_FUNC_SPI);
 
     // Setup busy line
     gpio_init(SAMWISE_RF_D0_PIN);
     gpio_set_dir(SAMWISE_RF_D0_PIN, GPIO_IN);
     gpio_pull_down(SAMWISE_RF_D0_PIN);
 
     rfm96_reset();
     busy_wait_ms(10);
     spi_pins->spi = global_spi;
 
     // RFM9X.pdf 4.3 p75:
     // CPOL = 0, CPHA = 0 (mode 0)
     // MSB first
    spi_init(global_spi, RFM96_SPI_BAUDRATE);
    // Set SPI bus details --RFM9X.pdf 4.3 p75: CPOL = 0, CPHA = 0 (mode 0) MSB first
    // This is also the pico-sdk default:spi_set_format(8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_format(global_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // 0x42 is the Chip ID register and the value returned should be 0x11
    uint8_t v = 0;
    printf((reg_read(0x42, &v, 1) == 1) ? "RFM9X Chip ID read success\n" : "RFM9X Chip ID read failed\n");
    printf((v != 0x11) ? "RFM9X version check success\n" : "RFM9X version 0x11 check failed, returned %02x\n", v);

     /*
      * Calibrate the oscillator
      */
     rfm96_set_mode(STANDBY_MODE);
     sleep_ms(10);
     rfm96_trigger_osc_calibration();
     sleep_ms(1000); // 1 second
 
     /*
      * Configure LoRa
      */
     rfm96_set_mode(SLEEP_MODE);
     sleep_ms(10);
     rfm96_set_lora(1);
 
     /*
      * Use entire FIFO for RX & TX
      * TODO: This seems bad for simultaneous RX & TX..
      */
     rfm96_put8(_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
     rfm96_put8(_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
 
     /*
      * Disable frequency hopping
      */
     rfm96_put8(_RH_RF95_REG_24_HOP_PERIOD, 0);
 
     rfm96_set_mode(STANDBY_MODE);
 
     /*
      * Configure tranceiver properties
      */
     rfm96_set_frequency(RFM96_FREQUENCY); /* Always */
 
     rfm96_set_preamble_length(8); /* 8 bytes matches Radiohead library */
     ASSERT(rfm96_get_preamble_length() == 8);
 
     rfm96_set_bandwidth(RFM96_BANDWIDTH); /* Configure 125000 to match
                                        Radiohead, see SX1276 errata note 2.3 */
     ASSERT(rfm96_get_bandwidth() == RFM96_BANDWIDTH);
 
     rfm96_set_coding_rate(5); /* Configure 4/5 to match Radiohead library */
     ASSERT(rfm96_get_coding_rate() == 5);
 
     rfm96_set_spreading_factor(7); /* Configure to 7 to match Radiohead library */
     ASSERT(rfm96_get_spreading_factor() == 7);
 
     rfm96_set_crc(1); /* Enable CRC checking */
     ASSERT(rfm96_is_crc_enabled() == 1);
 
     rfm96_put8(_RH_RF95_REG_26_MODEM_CONFIG3, 0x00); /* No sync word */
     rfm96_set_tx_power(15);                          /* Known good value */
     ASSERT(rfm96_get_tx_power() == 15);
 
     rfm96_set_pa_ramp(0);
     ASSERT(rfm96_get_pa_ramp() == 0);
 
     rfm96_set_lna_boost(0b11);
     ASSERT(rfm96_get_lna_boost() == 0b11);
     printf("rfm96: Initialization complete\n");
     return (v != 0x11);    // 0 is success; 1 is failure; 0x11 is the Chip ID again
 }

 //  Chip ID
 uint32_t rfm96_version(spi_pins_t spi_pins)
 {
     return (uint32_t)rfm96_get8(_RH_RF95_REG_42_VERSION);
 }
 
 void rfm96_transmit()
 {
     // we do not have an LNA
     rfm96_set_mode(TX_MODE);
     uint8_t dioValue = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
     dioValue = bits_set(dioValue, 6, 7, 0b00);
     rfm96_put8(_RH_RF95_REG_40_DIO_MAPPING1, dioValue);
 }
 
 void rfm96_listen()
 {
     rfm96_set_mode(RX_MODE);
     uint8_t dioValue = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
     dioValue = bits_set(dioValue, 6, 7, 0b00);
     rfm96_put8(_RH_RF95_REG_40_DIO_MAPPING1, dioValue);
 }
 
 uint8_t rfm96_tx_done()
 {
     return (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x8) >> 3;
 }
 
 uint8_t rfm96_rx_done()
 {
     uint8_t dioValue = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
     if (dioValue)
     {
         return dioValue;
     }
     else
     {
         return (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x40) >> 6;
     }
 }
 
 int rfm96_await_rx()
 {
     rfm96_listen();
     while (!rfm96_rx_done())
         ; // spin until RX done
     return 1;
 }
 
 //Outgoing payload
 uint8_t rfm96_packet_to_fifo(uint8_t *buf, uint8_t n)
 {
     uint8_t old_mode = rfm96_get_mode();
     rfm96_set_mode(STANDBY_MODE);
 
     rfm96_put8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
 
     rfm96_put_buf(_RH_RF95_REG_00_FIFO, buf, n);
     rfm96_put8(_RH_RF95_REG_22_PAYLOAD_LENGTH, n);
 
     rfm96_set_mode(old_mode);
     return 0;
 }
 
 //Incoming payload
 uint8_t rfm96_packet_from_fifo(uint8_t *buf)
 {
     uint8_t n_read = 0;
     uint8_t old_mode = rfm96_get_mode();
     rfm96_set_mode(STANDBY_MODE);
 
     // Check for CRC error
     if (rfm96_is_crc_enabled() && rfm96_crc_error())
     {
         // TODO report somehow
     }
     else
     {
         uint8_t fifo_length = rfm96_get8(_RH_RF95_REG_13_RX_NB_BYTES);
         if (fifo_length > 0)
         {
             uint8_t current_addr =
                 rfm96_get8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);
             rfm96_put8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, current_addr);
 
             // read the packet
             rfm96_get_buf(_RH_RF95_REG_00_FIFO, buf, fifo_length);
         }
         n_read = fifo_length;
     }
     rfm96_set_mode(old_mode);
     return n_read;
 }
