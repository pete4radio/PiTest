#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "pins.h"

/*
 * RFM96 Radio Driver - Non-Blocking DMA SPI Implementation
 *
 * This driver uses fully non-blocking DMA-based SPI transactions with timeouts.
 *
 * Key Features:
 * - Single-phase DMA transactions: Command + data/dummies in one buffer
 * - Simultaneous TX/RX DMA: Automatic echo byte capture eliminates FIFO drain polling
 * - CPU-yielding timeouts: sleep_us(1) polling instead of tight_loop_contents()
 * - No CS delays: GPIO write speed (30ns) exceeds datasheet requirement (5-10ns)
 * - Configurable debug logging: ERROR, INFO, VERBOSE levels
 *
 * Performance Improvements vs. Blocking Version:
 * - Single register read/write: 2-3x faster (60-80us vs 100-200us)
 * - 256-byte FIFO transfer: 20-30% faster (500-550us vs 600-800us)
 * - CPU load during DMA: ~0.1% (yields) vs 100% (tight loop)
 *
 * SPI Configuration:
 * - Full-duplex: Every TX byte generates an RX echo byte
 * - Baud rate: 5 MHz
 * - Mode: CPOL=0, CPHA=0 (Motorola/Freescale)
 * - Timeout: 20ms per DMA transaction (48x safety margin)
 *
 * Register Access Modes:
 * - SINGLE: Address + 1 data byte
 * - BURST: Address + multiple consecutive bytes
 * - FIFO: Address 0x00 + sequential FIFO access
 */

// Strategy differs from flight software.   Check only  the radio functionality & do range test:
//   o  SPI is used everywhere, so lets just make it a global.  CS is used only in init and 
// at lowest level routines and it's clumsy to pass it around.  Use SAMWISE_RF_CS_PIN.
//   o  pass by value (except buffers) to avoid corruption and nasty bugs later (this code needs
// to be easy to use and modify).
//   o  Non-blocking calls with timeout for DMA transfers to avoid hangs.
//   o  For our purposes, a packet is a received transmission (no src and dest)
// 

#ifdef PICO
    spi_inst_t *global_spi = spi0;
#else
    spi_inst_t *global_spi = spi1;
#endif

// DMA Debug logging levels
#define DMA_DEBUG_NONE 0
#define DMA_DEBUG_ERROR 1    // Errors/timeouts only
#define DMA_DEBUG_INFO 2     // + transaction start/complete
#define DMA_DEBUG_VERBOSE 3  // + data dumps

#define DMA_DEBUG DMA_DEBUG_ERROR

// Logging macros
#if DMA_DEBUG >= DMA_DEBUG_ERROR
    #define DMA_LOG_ERROR(...) printf(__VA_ARGS__)
#else
    #define DMA_LOG_ERROR(...)
#endif

#if DMA_DEBUG >= DMA_DEBUG_INFO
    #define DMA_LOG_INFO(...) printf(__VA_ARGS__)
#else
    #define DMA_LOG_INFO(...)
#endif

#if DMA_DEBUG >= DMA_DEBUG_VERBOSE
    #define DMA_LOG_VERBOSE(...) printf(__VA_ARGS__)
#else
    #define DMA_LOG_VERBOSE(...)
#endif

// DMA channels for SPI transfers
static int tx_dma_chan = -1;
static int rx_dma_chan = -1;

// Timeout for DMA transfers in microseconds (20ms)
#define DMA_TIMEOUT_US 20000

// Timeout tracking structure
typedef struct {
    uint64_t start_time_us;
    uint32_t timeout_us;
    const char *operation_name;
} transaction_timeout_t;

// Static buffers for DMA transactions
static uint8_t rx_discard_small[2];      // For 1-2 byte echoes
static uint8_t rx_discard_large[257];    // For large writes (cmd + 256 data)
static uint8_t tx_combined_small[3];     // For small combined cmd + dummies
static uint8_t tx_combined_large[257];   // For combined cmd + dummy bytes
static uint8_t rx_combined_small[3];     // For small echo + data
static uint8_t rx_combined_large[257];   // For combined echo + data

#define DEBUG 0

/*******************************************************************************
 * Function Declarations
 */
// Timeout tracking helpers
static inline void timeout_start(transaction_timeout_t *t, uint32_t timeout_us, const char *op_name);
static inline bool timeout_check(transaction_timeout_t *t);
static inline void timeout_report(transaction_timeout_t *t);

// DMA wait functions
int dma_wait_with_timeout(int channel);
int dma_wait_both_with_timeout(transaction_timeout_t *timeout_ctx);

void reg_write(const uint8_t reg, const uint8_t data);

int reg_read(const uint8_t reg, uint8_t *buf, uint8_t nbytes);

void rfm96_reset();

int rfm96_init(spi_pins_t *spi_pins);

/*******************************************************************************
* Function Definitions
*/

// Timeout tracking helper functions
static inline void timeout_start(transaction_timeout_t *t, uint32_t timeout_us, const char *op_name) {
    t->start_time_us = time_us_64();
    t->timeout_us = timeout_us;
    t->operation_name = op_name;
}

static inline bool timeout_check(transaction_timeout_t *t) {
    return (time_us_64() - t->start_time_us) >= t->timeout_us;
}

static inline void timeout_report(transaction_timeout_t *t) {
    uint64_t elapsed = time_us_64() - t->start_time_us;
    DMA_LOG_ERROR("SPI TIMEOUT: %s failed after %llu us (limit: %lu us)\n",
                  t->operation_name, elapsed, t->timeout_us);
}

// Wait for DMA channel to complete with timeout
// Returns 0 on success, -1 on timeout
int dma_wait_with_timeout(int channel) {
    uint64_t start_time = time_us_64();

    while (dma_channel_is_busy(channel)) {
        uint64_t elapsed = time_us_64() - start_time;
        if (elapsed > DMA_TIMEOUT_US) {
            // Timeout occurred - abort the DMA transfer
            dma_channel_abort(channel);
            DMA_LOG_ERROR("DMA timeout on channel %d after %llu us\n", channel, elapsed);
            return -1;
        }
        // Yield CPU with 1us sleep for maximum responsiveness
        sleep_us(1);
    }

    return 0;
}

// Wait for both TX and RX DMA channels to complete with timeout
// Returns 0 on success, -1 on timeout
int dma_wait_both_with_timeout(transaction_timeout_t *timeout_ctx) {
    while (dma_channel_is_busy(tx_dma_chan) || dma_channel_is_busy(rx_dma_chan)) {
        if (timeout_check(timeout_ctx)) {
            dma_channel_abort(tx_dma_chan);
            dma_channel_abort(rx_dma_chan);
            timeout_report(timeout_ctx);
            return -1;
        }
        // Yield CPU with 1us sleep for maximum responsiveness
        sleep_us(1);
    }
    return 0;
}

// Write 1 byte to the specified register
void reg_write(const uint8_t reg, const uint8_t data) {
    transaction_timeout_t timeout;
    uint8_t msg[2];    // Need two bytes as we need to clock out the address and the data

    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x80 | reg;  // Set write bit
    msg[1] = data;

    DMA_LOG_INFO("reg_write: Writing 0x%02x to register 0x%02x\n", data, reg);
    DMA_LOG_VERBOSE("  TX: [0x%02x 0x%02x]\n", msg[0], msg[1]);

    // Ensure TX DMA is configured for writes (read_increment = true)
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(global_spi, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure(tx_dma_chan, &tx_cfg, &spi_get_hw(global_spi)->dr,
                         NULL, 0, false);

    // CS is active low:  Write to register using DMA
    gpio_put(SAMWISE_RF_CS_PIN, 0);

    // Setup simultaneous TX and RX DMA transfers
    // RX DMA automatically captures echo bytes into discard buffer
    dma_channel_set_read_addr(tx_dma_chan, msg, false);
    dma_channel_set_trans_count(tx_dma_chan, 2, false);

    dma_channel_set_write_addr(rx_dma_chan, rx_discard_small, false);
    dma_channel_set_trans_count(rx_dma_chan, 2, false);

    // Start both channels simultaneously
    dma_start_channel_mask((1u << tx_dma_chan) | (1u << rx_dma_chan));

    // Wait for completion with timeout
    timeout_start(&timeout, DMA_TIMEOUT_US, "reg_write");
    if (dma_wait_both_with_timeout(&timeout) != 0) {
        DMA_LOG_ERROR("reg_write: DMA timeout writing to register 0x%02x\n", reg);
    }

    gpio_put(SAMWISE_RF_CS_PIN, 1);

    DMA_LOG_VERBOSE("  RX echo: [0x%02x 0x%02x] (discarded)\n",
                    rx_discard_small[0], rx_discard_small[1]);
    DMA_LOG_INFO("reg_write: Complete\n");
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    transaction_timeout_t timeout;

    if (nbytes < 1) {
        return -1;
    }

    DMA_LOG_INFO("reg_read: Reading %d byte(s) from register 0x%02x\n", nbytes, reg);

    // Select appropriate buffers based on size
    uint8_t *tx_buf = (nbytes <= 2) ? tx_combined_small : tx_combined_large;
    uint8_t *rx_buf = (nbytes <= 2) ? rx_combined_small : rx_combined_large;

    // Single-phase approach: Combine command + dummy bytes in one buffer
    // Command byte: wnr bit = 0 for read
    tx_buf[0] = reg & 0x7F;  // Clear write bit for read
    memset(&tx_buf[1], 0x00, nbytes);  // Fill with dummy bytes

    DMA_LOG_VERBOSE("  TX: [0x%02x", tx_buf[0]);
    for (int i = 1; i <= nbytes && DMA_DEBUG >= DMA_DEBUG_VERBOSE; i++) {
        printf(" 0x%02x", tx_buf[i]);
    }
    DMA_LOG_VERBOSE("]\n");

    // Ensure TX DMA is configured for reads (read_increment = true)
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(global_spi, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure(tx_dma_chan, &tx_cfg, &spi_get_hw(global_spi)->dr,
                         NULL, 0, false);

    gpio_put(SAMWISE_RF_CS_PIN, 0);

    // Single simultaneous TX/RX DMA transaction
    dma_channel_set_read_addr(tx_dma_chan, tx_buf, false);
    dma_channel_set_trans_count(tx_dma_chan, 1 + nbytes, false);

    dma_channel_set_write_addr(rx_dma_chan, rx_buf, false);
    dma_channel_set_trans_count(rx_dma_chan, 1 + nbytes, false);

    // Start both channels simultaneously
    dma_start_channel_mask((1u << tx_dma_chan) | (1u << rx_dma_chan));

    // Wait for both to complete
    timeout_start(&timeout, DMA_TIMEOUT_US, "reg_read");
    if (dma_wait_both_with_timeout(&timeout) != 0) {
        DMA_LOG_ERROR("reg_read: DMA timeout reading from register 0x%02x\n", reg);
        gpio_put(SAMWISE_RF_CS_PIN, 1);
        return -1;
    }

    gpio_put(SAMWISE_RF_CS_PIN, 1);

    // Copy data (skip first byte = command echo)
    memcpy(buf, &rx_buf[1], nbytes);

    DMA_LOG_VERBOSE("  RX: [0x%02x (echo)", rx_buf[0]);
    for (int i = 0; i < nbytes && DMA_DEBUG >= DMA_DEBUG_VERBOSE; i++) {
        printf(" 0x%02x", buf[i]);
    }
    DMA_LOG_VERBOSE("]\n");
    DMA_LOG_INFO("reg_read: Complete\n");

    return nbytes;
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
     gpio_put(SAMWISE_RF_CS_PIN, 0);    //CS is active low
     // No delay needed: GPIO write (~30ns) >> datasheet requirement (5-10ns)
 }

void cs_deselect()
 {
     gpio_put(SAMWISE_RF_CS_PIN, 1);
     // No delay needed: GPIO write (~30ns) >> datasheet requirement (5-10ns)
 }
 
 /*
  * Read a buffer from a register address.
  */
void rfm96_get_buf(rfm96_reg_t reg, uint8_t *buf, uint32_t n)
 {
     transaction_timeout_t timeout;

     DMA_LOG_INFO("rfm96_get_buf: Reading %lu byte(s) from register 0x%02x\n", n, reg);

     // Use large buffer for FIFO and burst reads
     uint8_t *tx_buf = tx_combined_large;
     uint8_t *rx_buf = rx_combined_large;

     // Single-phase approach: Combine command + dummy bytes in one buffer
     // Command byte: wnr bit = 0 for read
     tx_buf[0] = reg & 0x7F;  // Clear write bit for read
     memset(&tx_buf[1], 0x00, n);  // Fill with dummy bytes

     DMA_LOG_VERBOSE("  Reading FIFO/burst: cmd=0x%02x, nbytes=%lu\n", tx_buf[0], n);

     // Ensure TX DMA is configured for reads (read_increment = true)
     dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
     channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
     channel_config_set_dreq(&tx_cfg, spi_get_dreq(global_spi, true));
     channel_config_set_read_increment(&tx_cfg, true);
     channel_config_set_write_increment(&tx_cfg, false);
     dma_channel_configure(tx_dma_chan, &tx_cfg, &spi_get_hw(global_spi)->dr,
                          NULL, 0, false);

     cs_select();

     // Single simultaneous TX/RX DMA transaction
     dma_channel_set_read_addr(tx_dma_chan, tx_buf, false);
     dma_channel_set_trans_count(tx_dma_chan, 1 + n, false);

     dma_channel_set_write_addr(rx_dma_chan, rx_buf, false);
     dma_channel_set_trans_count(rx_dma_chan, 1 + n, false);

     // Start both channels simultaneously
     dma_start_channel_mask((1u << tx_dma_chan) | (1u << rx_dma_chan));

     // Wait for both to complete
     timeout_start(&timeout, DMA_TIMEOUT_US, "rfm96_get_buf");
     if (dma_wait_both_with_timeout(&timeout) != 0) {
         DMA_LOG_ERROR("rfm96_get_buf: DMA timeout reading buffer\n");
         cs_deselect();
         return;
     }

     cs_deselect();

     // Copy data (skip first byte = command echo)
     memcpy(buf, &rx_buf[1], n);

     DMA_LOG_VERBOSE("  First few bytes: [0x%02x 0x%02x 0x%02x ...]\n",
                     buf[0], n > 1 ? buf[1] : 0, n > 2 ? buf[2] : 0);
     DMA_LOG_INFO("rfm96_get_buf: Complete\n");
 }
 
 /*
  * Write a buffer to a register address.
  */
  void rfm96_put_buf(rfm96_reg_t reg, uint8_t *buf, uint32_t n)
 {
     transaction_timeout_t timeout;

     DMA_LOG_INFO("rfm96_put_buf: Writing %lu byte(s) to register 0x%02x\n", n, reg);
     DMA_LOG_VERBOSE("  First few bytes: [0x%02x 0x%02x 0x%02x ...]\n",
                     buf[0], n > 1 ? buf[1] : 0, n > 2 ? buf[2] : 0);

     // Use large buffer for FIFO and burst writes
     uint8_t *tx_buf = tx_combined_large;

     // Combine command + data in one buffer
     // Command byte: wnr bit = 1 for write
     tx_buf[0] = reg | 0x80;  // Set write bit
     memcpy(&tx_buf[1], buf, n);  // Copy data after command

     // Ensure TX DMA is configured for writes (read_increment = true)
     dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
     channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
     channel_config_set_dreq(&tx_cfg, spi_get_dreq(global_spi, true));
     channel_config_set_read_increment(&tx_cfg, true);
     channel_config_set_write_increment(&tx_cfg, false);
     dma_channel_configure(tx_dma_chan, &tx_cfg, &spi_get_hw(global_spi)->dr,
                          NULL, 0, false);

     cs_select();

     // Setup simultaneous TX and RX DMA transfers
     // RX DMA automatically captures echo bytes into discard buffer
     dma_channel_set_read_addr(tx_dma_chan, tx_buf, false);
     dma_channel_set_trans_count(tx_dma_chan, 1 + n, false);

     dma_channel_set_write_addr(rx_dma_chan, rx_discard_large, false);
     dma_channel_set_trans_count(rx_dma_chan, 1 + n, false);

     // Start both channels simultaneously
     dma_start_channel_mask((1u << tx_dma_chan) | (1u << rx_dma_chan));

     // Wait for completion with timeout
     timeout_start(&timeout, DMA_TIMEOUT_US, "rfm96_put_buf");
     if (dma_wait_both_with_timeout(&timeout) != 0) {
         DMA_LOG_ERROR("rfm96_put_buf: DMA timeout writing buffer\n");
         cs_deselect();
         return;
     }

     cs_deselect();

     DMA_LOG_VERBOSE("  RX echo bytes automatically discarded (%lu bytes)\n", 1 + n);
     DMA_LOG_INFO("rfm96_put_buf: Complete\n");
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
  * Must be done in sleep mode, exits in operation mode.
  */
  void rfm96_set_lora(uint8_t lora)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     printf("rfm96_set_lora: Read OpMode reg=0x%02x\n", reg);
     if (lora)
         reg = bit_set(reg, 7);
     else
         {
            reg = bit_clr(reg, 7);
            printf("rfm96_set_lora: Warning, LoRa mode was set off\n");
         }
     printf("rfm96_set_lora: Writing OpMode reg=0x%02x\n", reg);
     rfm96_put8(_RH_RF95_REG_01_OP_MODE, reg);
 }
 
 /*
  * Get long range mode (LoRa status)
  * (RFM9X.pdf 6.2 p87)
  */
  uint8_t rfm96_get_lora()
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_01_OP_MODE);
     printf("rfm96_get_lora: Read OpMode reg=0x%02x, bit7=%d\n", reg, bit_is_on(reg, 7));
     return bit_is_on(reg, 7);
 }
 
/*Set Low Data Rate Optimization
    * (RFM9X.pdf 6.4 p107)

  */
 void rfm96_set_ldro(uint8_t ldro)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_26_MODEM_CONFIG3);
     if (ldro)
         reg = bit_set(reg, 2);
     else
         {
            reg = bit_clr(reg, 2);
            printf("rfm96_set_ldro: Warning, LDRO was set off\n");
         }
     rfm96_put8(_RH_RF95_REG_26_MODEM_CONFIG3, reg);
 }
 
 /*
  * Get Low Data Rate Optimization (MobileNode)
  * (RFM9X.pdf 6.4 p107)
  */
  uint8_t rfm96_get_LDRO()
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_26_MODEM_CONFIG3);
     return bit_is_on(reg, 2);
 }

/* Set Auto AGC
    (RFM9X.pdf 6.4 p107)
  */
 void rfm96_set_AGC(uint8_t agc)
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_26_MODEM_CONFIG3);
     if (agc)
         reg = bit_set(reg, 3);
     else
         {
            reg = bit_clr(reg, 3);
            printf("rfm96_set_agc: Warning, Auto AGC was set off\n");
         }
     rfm96_put8(_RH_RF95_REG_26_MODEM_CONFIG3, reg);
 }
 
 /*
  * Get AGC

  */
  uint8_t rfm96_get_agc()
 {
     uint8_t reg = rfm96_get8(_RH_RF95_REG_26_MODEM_CONFIG3);
     return bit_is_on(reg, 3);
 }

 /*
  * Triggers oscillator calibration (RFM9X.pdf 6.2 p93)
  *
  * Must be done outside of LoRa mode, since register 0x24 is aliased.
  * This is done automatically when the chip is powered up.
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
    //  compute with high precision and cast back to uint32_t
    uint32_t frf = ((uint32_t)((double)f / ((double)32000000 / (double) 524288))) & 0xFFFFFF;
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
  * Uses same high-precision calculation as setter to ensure ASSERT passes
  */
  uint32_t rfm96_get_frequency()
 {
     uint32_t msb = rfm96_get8(_RH_RF95_REG_06_FRF_MSB);
     uint32_t mid = rfm96_get8(_RH_RF95_REG_07_FRF_MID);
     uint32_t lsb = rfm96_get8(_RH_RF95_REG_08_FRF_LSB);
     uint32_t frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF;
     // Use high-precision inverse calculation matching the setter
     return (uint32_t)((double)frf * ((double)32000000 / (double)524288));
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
 /*
  * Set TX power in dBm
  * Based on Adafruit CircuitPython RFM9x and RadioHead RH_RF95 libraries
  *
  * Low power mode (RFO pin): -1 to 14 dBm
  * High power mode (PA_BOOST): 5 to 23 dBm (>20 dBm uses PA_DAC boost)
  *
  * Current implementation uses low power mode (RFO) for laptop power compatibility
  */
 void rfm96_set_tx_power(int8_t power)
 {
     if (0)     // high power mode (PA_BOOST) - out of reach when laptop-powered
     {
         // Constrain to valid range for high power
         if (power > 23)
             power = 23;
         if (power < 5)
             power = 5;

         // Enable PA_DAC boost for power > 20 dBm (adds ~3dBm)
         // Per datasheet, reduce setting by 3dB when PA_BOOST enabled
         if (power > 20)
         {
             rfm96_set_pa_dac(_RH_RF95_PA_DAC_ENABLE);
             power -= 3;  // Compensate for PA_DAC boost
         }
         else
         {
             rfm96_set_pa_dac(_RH_RF95_PA_DAC_DISABLE);
         }

         // Use PA_BOOST pin (bit 7 = 1)
         rfm96_set_pa_output_pin(1);
         rfm96_set_max_power(0b111);
         // Formula: Pout = 17 - (15 - OutputPower), simplified to: OutputPower = power - 2
         // But RadioHead uses: Pout = 2 + OutputPower, so: OutputPower = power - 5
         rfm96_set_raw_tx_power((power - 5) & 0x0F);
     }
     else
     {
         // Low/Medium power mode using PA_BOOST: 2 to 17 dBm
         // (Uses PA_BOOST pin but without PA_DAC boost)
         // Constrain to valid range
         if (power > 17)
             power = 17;
         if (power < 2)
             power = 2;

         // Disable PA_DAC boost (normal PA_BOOST mode)
         rfm96_set_pa_dac(_RH_RF95_PA_DAC_DISABLE);

         // Use PA_BOOST pin (bit 7 = 1)
         rfm96_set_pa_output_pin(1);
         rfm96_set_max_power(0b111);
         // Formula for PA_BOOST without PA_DAC: Pout = 2 + OutputPower
         // So: OutputPower = power - 2
         rfm96_set_raw_tx_power((power - 2) & 0x0F);
     }
 }
 
 /*
  * Get the TX power in dBm
  * Based on Adafruit CircuitPython RFM9x and RadioHead RH_RF95 libraries
  *
  * Returns power level by reading PA_CONFIG register and inverting the setter's calculation
  * Low power mode (RFO): returns -1 to 14 dBm
  * High power mode (PA_BOOST): returns 5 to 23 dBm (accounts for PA_DAC boost if enabled)
  */
 int8_t rfm96_get_tx_power()
 {
     uint8_t pa_config = rfm96_get8(_RH_RF95_REG_09_PA_CONFIG);
     uint8_t pa_select = (pa_config >> 7) & 0x01;  // Bit 7: PA_SELECT
     uint8_t output_power = pa_config & 0x0F;      // Bits 3-0: OutputPower

     if (pa_select)  // PA_BOOST mode
     {
         // Check if PA_DAC boost is enabled
         uint8_t pa_dac = rfm96_get8(_RH_RF95_REG_4D_PA_DAC);

         if (pa_dac == _RH_RF95_PA_DAC_ENABLE)
         {
             // High power mode with PA_DAC: Pout = 5 + OutputPower (17-20 dBm base)
             // Setter uses: OutputPower = (power - 3 - 5), so power = OutputPower + 5 + 3
             return output_power + 5 + 3;
         }
         else
         {
             // Normal PA_BOOST mode without PA_DAC: Pout = 2 + OutputPower
             // Inverse of setter: OutputPower = (power - 2)
             // So: power = OutputPower + 2
             return output_power + 2;
         }
     }
     else  // RFO mode (not used in current config, but included for completeness)
     {
         // RFO mode: Pout = Pmax - (15 - OutputPower)
         // Inverse of setter: OutputPower = (power + 1)
         // So: power = OutputPower - 1
         return (int8_t)output_power - 1;
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
//  and sets up the radio for LoRa mode.  It also calibrates the oscillator and
//  sets the frequency to RFM96_FREQUENCY and turns on the receiver.
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

    // Initialize DMA channels for SPI
    tx_dma_chan = dma_claim_unused_channel(true);
    rx_dma_chan = dma_claim_unused_channel(true);

    // Configure TX DMA: memory -> SPI TX FIFO
    // Default configuration is for reads (sending dummy bytes with read_increment = false)
    // Write functions will reconfigure this as needed
    dma_channel_config tx_config = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_dreq(&tx_config, spi_get_dreq(global_spi, true));
    channel_config_set_read_increment(&tx_config, false);  // For dummy byte reads
    channel_config_set_write_increment(&tx_config, false);
    dma_channel_configure(
        tx_dma_chan,
        &tx_config,
        &spi_get_hw(global_spi)->dr,  // Write to SPI TX FIFO
        NULL,                          // Read address set per transfer
        0,                             // Transfer count set per transfer
        false                          // Don't start yet
    );

    // Configure RX DMA: SPI RX FIFO -> memory
    dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_dreq(&rx_config, spi_get_dreq(global_spi, false));
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    dma_channel_configure(
        rx_dma_chan,
        &rx_config,
        NULL,                          // Write address set per transfer
        &spi_get_hw(global_spi)->dr,  // Read from SPI RX FIFO
        0,                             // Transfer count set per transfer
        false                          // Don't start yet
    );

    printf("DMA channels initialized: TX=%d, RX=%d\n", tx_dma_chan, rx_dma_chan);

    // 0x42 is the Chip ID register and the value returned should be 0x12
    uint8_t v = 0;
    printf((reg_read(0x42, &v, 1) == 1) ? "RFM9X Chip ID read success\n" : "RFM9X Chip ID read failed\n");
    printf((v == 0x12) ? "RFM9X version check success\n" : "RFM9X version 0x12 check failed, returned %02x\n", v);

    // Setup busy line, which signals when RX packet received or TX packet sent
     gpio_init(SAMWISE_RF_D0_PIN);
     gpio_set_dir(SAMWISE_RF_D0_PIN, GPIO_IN);
     gpio_pull_down(SAMWISE_RF_D0_PIN);
   
     /*
      * Calibrate the oscillator
      */
     //rfm96_set_mode(STANDBY_MODE);
     //sleep_ms(10);
     //rfm96_trigger_osc_calibration();
     //sleep_ms(1000); // 1 second
 
     /*
      * Configure LoRa
      */
     rfm96_set_mode(SLEEP_MODE);
     sleep_ms(10);
     ASSERT(rfm96_get_mode() == SLEEP_MODE);
     rfm96_set_lora(1);
     sleep_ms(10);
    ASSERT(rfm96_get_lora() == 1);
     // check for SLEEP_MODE and LoRa mode
        if((rfm96_get_mode() != SLEEP_MODE) || (rfm96_get_lora() != 1))
        {
            printf("rfm96: ERROR not in sleep mode or LoRa mode\n");
        }

     /*
      * Use entire FIFO for RX & TX.  We run half-duplex, so we don't  
      * use it for RX when receiving and for TX when transmitting.
      * The FIFO is 256 bytes long, so we set the base address to 0.
      * The FIFO pointer is set to 0, so the first byte written to the FIFO
      * will be at address 0.
      */
     rfm96_set_mode(STANDBY_MODE);
     rfm96_put8(_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
     rfm96_put8(_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
 
     /*
      * Disable frequency hopping
      */
     //rfm96_put8(_RH_RF95_REG_24_HOP_PERIOD, 0);
 
     /*
      * Configure tranceiver properties
      */
     rfm96_set_frequency(RFM96_FREQUENCY); /* Always */
     printf("rfm96: Desired frequency minus get frequency is %d Hz\n", RFM96_FREQUENCY - rfm96_get_frequency());
     ASSERT(rfm96_get_frequency() == RFM96_FREQUENCY);
 
     rfm96_set_preamble_length(8); /* 8 bytes matches Radiohead library */
     ASSERT(rfm96_get_preamble_length() == 8);
 
     rfm96_set_bandwidth(RFM96_BANDWIDTH); /* Configure 125000 to match
                                        Radiohead, see SX1276 errata note 2.3 */
     ASSERT(rfm96_get_bandwidth() == RFM96_BANDWIDTH);
 
     rfm96_set_coding_rate(8); /* Configure 4/8 to match Giganteum */
     ASSERT(rfm96_get_coding_rate() == 8);
 
     rfm96_set_spreading_factor(7); /* Configure to 7 to match Radiohead library */
     ASSERT(rfm96_get_spreading_factor() == 7);
 
     rfm96_set_crc(1); /* Enable CRC sending */
     ASSERT(rfm96_is_crc_enabled() == 1);
 
     //rfm96_put8(_RH_RF95_REG_26_MODEM_CONFIG3, 0x00); /* No sync word */

        rfm96_set_ldro(1); /* Enable low data rate optimization */
        ASSERT(rfm96_get_LDRO() == 1);
        rfm96_set_AGC(1); /* Enable auto AGC */
        ASSERT(rfm96_get_agc() == 1);

     rfm96_set_tx_power(15);                          /* Known good value */
     ASSERT(rfm96_get_tx_power() == 15);
 
     rfm96_set_pa_ramp(0);
     ASSERT(rfm96_get_pa_ramp() == 0);
 
     rfm96_set_lna_boost(0b11);
     ASSERT(rfm96_get_lna_boost() == 0b11);

//  Enable the busy I/O line to signal RX and TX done
//        uint8_t dio_mapping1 = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
//        rfm96_put8(_RH_RF95_REG_40_DIO_MAPPING1, dio_mapping1 & 0b00111111); // DIO0 = RXDONE/TXDONE

     rfm96_listen();       // Start by listening
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
//  Arm TX interrupts
    uint8_t dioValue = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
    dioValue = bits_set(dioValue, 6, 7, 0b01);   //IRQ when TX is done please
    rfm96_put8(_RH_RF95_REG_40_DIO_MAPPING1, dioValue);
    // Clear any pending interrupts, they might be from the receiver
    rfm96_put8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
    sleep_us(10);  // Brief delay to ensure IRQ flags are fully cleared before reading back
    if (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) != 0x00) {printf("rfm96_transmit: IRQ flags not successfully cleared\n");};
//  Going on the air!
    rfm96_set_mode(TX_MODE);
 }
 
 void rfm96_listen()
 {
    uint8_t dioValue = rfm96_get8(_RH_RF95_REG_40_DIO_MAPPING1);
    dioValue = bits_set(dioValue, 6, 7, 0b00);      //IRQ when RX is done please
    rfm96_put8(_RH_RF95_REG_40_DIO_MAPPING1, dioValue);
// Clear any pending interrupts, they might be from the transmitter
    rfm96_put8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
    sleep_us(10);  // Brief delay to ensure IRQ flags are fully cleared before reading back
    if (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) != 0x00) {printf("rfm96_listen: IRQ flags not successfully cleared\n");};
//  Put incoming packet at bottom of FIFO
    rfm96_put8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
    rfm96_put8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, 0x00);

    // Set RX mode
    rfm96_set_mode(RX_MODE);
 }
 
 uint8_t rfm96_tx_done()
 {
    // Check the TX_DONE bit (bit 3, 0x08) in the IRQ_FLAGS register
    // LoRa datasheet Table 18: IRQ flags
    return (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x08) >> 3;
}

 uint8_t rfm96_rx_done()
 {
    // Check the RX_DONE bit (bit 6, 0x40) in the IRQ_FLAGS register
    // LoRa datasheet Table 18: IRQ flags
    return (rfm96_get8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x40) >> 6;
 }

 /*
  * Get SNR (Signal-to-Noise Ratio) of last received packet
  * Returns SNR in dB (signed, can be negative)
  * Formula from datasheet: SNR = PacketSnr / 4 (in dB)
  */
 int8_t rfm96_get_snr()
 {
     int8_t snr_raw = (int8_t)rfm96_get8(_RH_RF95_REG_19_PKT_SNR_VALUE);
     return snr_raw / 4;  // Datasheet: SNR = PacketSnr[7:0] / 4
 }

 /*
  * Get RSSI (Received Signal Strength Indicator) of last received packet
  * Returns RSSI in dBm
  * Formula from datasheet section 5.5.5:
  * - If SNR >= 0: RSSI = -157 + PacketRssi
  * - If SNR < 0:  RSSI = -157 + PacketRssi + PacketSnr*0.25
  */
 int16_t rfm96_get_rssi()
 {
     int8_t snr_raw = (int8_t)rfm96_get8(_RH_RF95_REG_19_PKT_SNR_VALUE);
     uint8_t rssi_raw = rfm96_get8(_RH_RF95_REG_1A_PKT_RSSI_VALUE);

     int16_t rssi = -157 + rssi_raw;
     if (snr_raw < 0) {
         rssi += (snr_raw / 4);  // Adjust for negative SNR
     }
     return rssi;
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
//   uint8_t old_mode = rfm96_get_mode();
     rfm96_set_mode(STANDBY_MODE);
 
     rfm96_put8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00);
 
     rfm96_put_buf(_RH_RF95_REG_00_FIFO, buf, n);
     rfm96_put8(_RH_RF95_REG_22_PAYLOAD_LENGTH, n);
 
//   rfm96_set_mode(old_mode);
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
         printf("rfm96: got a packet but it has a CRC error\n");
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
         n_read = MIN(fifo_length, 255);
     }
     rfm96_set_mode(old_mode);
     return n_read;
 }
