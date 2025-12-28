#include "sband.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

/*
 * SX1280 SBand Radio Driver - DMA-based SPI Implementation
 *
 * Key differences from SX1276/RFM96:
 * - 16-bit register addresses (vs 8-bit)
 * - Different command set (0x80-0xC1 range)
 * - BUSY pin (D0) polled before SPI commands with 20ms timeout
 * - IRQ status is 16-bit (vs 8-bit)
 * - Frequency range 2.4 GHz (vs 437 MHz)
 * - Shared SPI bus with UHF radio
 */

// Enable/disable setter verification (compile-time flag)
#ifndef SBAND_VERIFY_SETTERS
    #define SBAND_VERIFY_SETTERS 1  // Default: enabled
#endif

#if SBAND_VERIFY_SETTERS
    #include <assert.h>

    #define SBAND_ASSERT_MODE(expected) do { \
        sx1280_mode_t actual = sband_get_mode(); \
        if (actual != (expected)) { \
            printf("SBand ASSERT FAILED: Expected mode %d, got %d\n", (expected), actual); \
            assert(actual == (expected)); \
        } \
    } while(0)

    #define SBAND_ASSERT_PACKET_TYPE(expected) do { \
        sx1280_packet_type_t actual = sband_get_packet_type(); \
        if (actual != (expected)) { \
            printf("SBand ASSERT FAILED: Expected packet type %d, got %d\n", (expected), actual); \
            assert(actual == (expected)); \
        } \
    } while(0)
#else
    #define SBAND_ASSERT_MODE(expected) ((void)0)
    #define SBAND_ASSERT_PACKET_TYPE(expected) ((void)0)
#endif

// SX1280 Commands
#define SX1280_CMD_GET_STATUS                0xC0
#define SX1280_CMD_WRITE_REGISTER            0x18
#define SX1280_CMD_READ_REGISTER             0x19
#define SX1280_CMD_WRITE_BUFFER              0x1A
#define SX1280_CMD_READ_BUFFER               0x1B
#define SX1280_CMD_SET_SLEEP                 0x84
#define SX1280_CMD_SET_STANDBY               0x80
#define SX1280_CMD_SET_FS                    0xC1
#define SX1280_CMD_SET_TX                    0x83
#define SX1280_CMD_SET_RX                    0x82
#define SX1280_CMD_SET_PACKET_TYPE           0x8A
#define SX1280_CMD_GET_PACKET_TYPE           0x03
#define SX1280_CMD_SET_RF_FREQUENCY          0x86
#define SX1280_CMD_SET_TX_PARAMS             0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS     0x8B
#define SX1280_CMD_SET_PACKET_PARAMS         0x8C
#define SX1280_CMD_SET_DIO_IRQ_PARAMS        0x8D
#define SX1280_CMD_SET_BUFFER_BASE_ADDRESS   0x8F
#define SX1280_CMD_GET_IRQ_STATUS            0x15
#define SX1280_CMD_CLEAR_IRQ_STATUS          0x97
#define SX1280_CMD_GET_RX_BUFFER_STATUS      0x17
#define SX1280_CMD_GET_PACKET_STATUS         0x1D
#define SX1280_CMD_GET_RSSI_INST             0x1F

// SX1280 Register addresses (16-bit addresses accessed via CMD_READ_REGISTER/CMD_WRITE_REGISTER)
#define SX1280_REG_VERSION_STRING           0x01F0  // 16-byte firmware version string
#define SX1280_REG_LORA_SYNC_WORD_MSB       0x0944  // LoRa sync word MSB
#define SX1280_REG_LORA_SYNC_WORD_LSB       0x0945  // LoRa sync word LSB
#define SX1280_REG_LORA_SF_CONFIG           0x0925  // SF configuration
#define SX1280_REG_LORA_RX_CODING_RATE      0x0950  // RX coding rate (readable after RX)

// Timeout for DMA transfers in microseconds (20ms)
#define DMA_TIMEOUT_US 20000

// SX1280 SPI baudrate (can go up to 18 MHz, using 5 MHz for reliability)
#define SX1280_SPI_BAUDRATE (5 * 1000 * 1000)

// Global SPI state
static spi_inst_t *sband_spi = NULL;
static uint8_t sband_cs_pin;
static uint8_t sband_rst_pin;
static uint8_t sband_d0_pin;
static uint8_t sband_rxen_pin;
static uint8_t sband_txen_pin;
static int sband_tx_dma_chan = -1;
static int sband_rx_dma_chan = -1;
static int radio_initialized = 0;

// DMA buffers
static uint8_t sband_tx_combined[258];
static uint8_t sband_rx_combined[258];

// BUSY polling timeout and counter
#define BUSY_TIMEOUT_US 20000
volatile uint32_t sband_busy_timeout_count = 0;

// Timeout tracking structure
typedef struct {
    uint64_t start_time_us;
    uint32_t timeout_us;
    const char *operation_name;
} sband_timeout_t;

/*******************************************************************************
 * Low-level SPI functions
 ******************************************************************************/

// Timeout tracking helpers
static inline void sband_timeout_start(sband_timeout_t *t, uint32_t timeout_us, const char *op_name) {
    t->start_time_us = time_us_64();
    t->timeout_us = timeout_us;
    t->operation_name = op_name;
}

static inline bool sband_timeout_check(sband_timeout_t *t) {
    return (time_us_64() - t->start_time_us) >= t->timeout_us;
}

// Wait for both TX and RX DMA channels to complete with timeout
static int sband_dma_wait_both_with_timeout(sband_timeout_t *timeout_ctx) {
    while (dma_channel_is_busy(sband_tx_dma_chan) || dma_channel_is_busy(sband_rx_dma_chan)) {
        if (sband_timeout_check(timeout_ctx)) {
            dma_channel_abort(sband_tx_dma_chan);
            dma_channel_abort(sband_rx_dma_chan);
            printf("SBand: SPI TIMEOUT: %s failed after %llu us (limit: %lu us)\n",
                   timeout_ctx->operation_name,
                   (time_us_64() - timeout_ctx->start_time_us),
                   timeout_ctx->timeout_us);
            return -1;
        }
        sleep_us(1);
    }
    return 0;
}

// Poll BUSY pin (D0) until LOW with timeout
// Returns true if still busy (timed out), false if BUSY went LOW
static bool sband_still_busy_after_wait(void) {
    uint64_t start_time = time_us_64();

    while (gpio_get(sband_d0_pin)) {  // While BUSY is HIGH
        if ((time_us_64() - start_time) >= BUSY_TIMEOUT_US) {
            sband_busy_timeout_count++;
            printf("SBand: BUSY timeout after %llu us\n",
                   (time_us_64() - start_time));
            radio_initialized = false;  // Mark radio as uninitialized on BUSY timeout
            return true;  // Still busy (timeout)
        }
        sleep_us(1);  // Small delay to prevent busy-waiting
    }

    return false;  // BUSY is now LOW (success)
}

// Low-level SPI transfer with DMA
static void sband_spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    sband_timeout_t timeout;

    // Wait for BUSY pin to go LOW before starting SPI transfer
    if (sband_still_busy_after_wait()) {
        printf("SBand: ERROR: BUSY timeout aborting SPI transfer\n");
        // Do not proceed - radio is marked as uninitialized
        return;
    }

    // Configure TX DMA
    dma_channel_config tx_cfg = dma_channel_get_default_config(sband_tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(sband_spi, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure(sband_tx_dma_chan, &tx_cfg, &spi_get_hw(sband_spi)->dr,
                         NULL, 0, false);

    // CS low
    gpio_put(sband_cs_pin, 0);

    // Start simultaneous TX/RX DMA
    dma_channel_set_read_addr(sband_tx_dma_chan, tx_buf, false);
    dma_channel_set_trans_count(sband_tx_dma_chan, len, false);

    dma_channel_set_write_addr(sband_rx_dma_chan, rx_buf, false);
    dma_channel_set_trans_count(sband_rx_dma_chan, len, false);

    // Start both channels
    dma_start_channel_mask((1u << sband_tx_dma_chan) | (1u << sband_rx_dma_chan));

    // Wait for completion
    sband_timeout_start(&timeout, DMA_TIMEOUT_US, "spi_transfer");
    sband_dma_wait_both_with_timeout(&timeout);

    // CS high
    gpio_put(sband_cs_pin, 1);
}

// Write command with data
static void sband_write_command(uint8_t cmd, const uint8_t *data, uint8_t len) {
    sband_tx_combined[0] = cmd;
    if (len > 0 && data != NULL) {
        memcpy(sband_tx_combined + 1, data, len);
    }
    sband_spi_transfer(sband_tx_combined, sband_rx_combined, len + 1);
}

// Read command with data
static void sband_read_command(uint8_t cmd, uint8_t *data, uint8_t len) {
    sband_tx_combined[0] = cmd;
    memset(sband_tx_combined + 1, 0x00, len + 1);  // NOP + dummy bytes
    sband_spi_transfer(sband_tx_combined, sband_rx_combined, len + 2);
    if (data != NULL && len > 0) {
        memcpy(data, sband_rx_combined + 2, len);  // Skip status + NOP
    }
}

// Write to register(s) using CMD_WRITE_REGISTER
// Protocol: [0x18] [addr_msb] [addr_lsb] [data0] [data1] ...
static void sband_write_register(uint16_t addr, const uint8_t *data, uint8_t len) {
    sband_tx_combined[0] = SX1280_CMD_WRITE_REGISTER;
    sband_tx_combined[1] = (addr >> 8) & 0xFF;  // Address MSB
    sband_tx_combined[2] = addr & 0xFF;         // Address LSB

    if (len > 0 && data != NULL) {
        memcpy(sband_tx_combined + 3, data, len);
    }

    sband_spi_transfer(sband_tx_combined, sband_rx_combined, len + 3);
}

// Read from register(s) using CMD_READ_REGISTER
// Protocol: [0x19] [addr_msb] [addr_lsb] [NOP...] → [status] [status] [status] [data0] ...
static void sband_read_register(uint16_t addr, uint8_t *data, uint8_t len) {
    sband_tx_combined[0] = SX1280_CMD_READ_REGISTER;
    sband_tx_combined[1] = (addr >> 8) & 0xFF;  // Address MSB
    sband_tx_combined[2] = addr & 0xFF;         // Address LSB

    // Fill with NOP (0x00) for dummy bytes
    memset(sband_tx_combined + 3, 0x00, len);

    // Transfer: 3 cmd/addr bytes + len data bytes
    sband_spi_transfer(sband_tx_combined, sband_rx_combined, len + 3);

    // Data starts at offset 3 (skip cmd + 2-byte address, status bytes embedded in response)
    if (data != NULL && len > 0) {
        memcpy(data, sband_rx_combined + 3, len);
    }
}

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

// Reset the SX1280
// Returns true if reset completed without BUSY timeouts, false otherwise
bool sband_reset(void) {
    bool success = true;

    // Check BUSY before reset
    if (sband_still_busy_after_wait()) {
        printf("SBand: ERROR: BUSY timeout before reset\n");
        success = false;
        // Continue anyway to attempt hardware recovery
    }

    // Perform hardware reset
    gpio_put(sband_rst_pin, 0);
    sleep_us(50);       // Hold reset low for at least 10us
    gpio_put(sband_rst_pin, 1);
    sleep_us(100);      // Wait for chip to boot (15us min)
    sleep_ms(50);       // Extra delay to ensure stability per logic analyzer tests

    // Check BUSY after reset
    if (sband_still_busy_after_wait()) {
        printf("SBand: ERROR: BUSY timeout after reset\n");
        success = false;
    }

    return success;
}

// Set operating mode
void sband_set_mode(sx1280_mode_t mode) {
    uint8_t cmd_data[1];
    uint8_t cmd;

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] WARNING: Attempting set mode despite BUSY timeout\n",
               __FILE__, __func__, __LINE__);
    }

    // Select the appropriate command based on mode
    uint8_t data_len = 0;

    switch(mode) {
        case SX1280_MODE_SLEEP:
            cmd = SX1280_CMD_SET_SLEEP;
            cmd_data[0] = 0x00;  // Sleep configuration
            data_len = 1;
            break;
        case SX1280_MODE_STDBY_RC:
        case SX1280_MODE_STDBY_XOSC:
            cmd = SX1280_CMD_SET_STANDBY;
            cmd_data[0] = (mode == SX1280_MODE_STDBY_RC) ? 0x00 : 0x01;
            data_len = 1;  // SET_STANDBY takes 1 byte
            break;
        case SX1280_MODE_FS:
            cmd = SX1280_CMD_SET_FS;
            data_len = 0;  // SET_FS is opcode-only
            break;
        default:
            // TX and RX modes should use sband_transmit() and sband_listen()
            printf("SBand: ERROR: Use sband_transmit() or sband_listen() for TX/RX modes\n");
            return;
    }

    sband_write_command(cmd, cmd_data, data_len);

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] WARNING: Returning from set mode despite BUSY timeout\n",
               __FILE__, __func__, __LINE__);
        printf("mode = %d; cmd = 0x%x; cmd_data[0] = 0x%x\n", mode, cmd, cmd_data[0]);
    }

}

// Set packet type
void sband_set_packet_type(sx1280_packet_type_t type) {
    uint8_t cmd_data[1] = {type};
    sband_write_command(SX1280_CMD_SET_PACKET_TYPE, cmd_data, 1);
}

// Set RF frequency
// freq_hz: Frequency in Hz (e.g., 2427000000 for 2427 MHz)
void sband_set_rf_frequency(uint32_t freq_hz) {
    // SX1280 frequency calculation: freq_reg = (freq_hz * 2^18) / 52000000
    // For 2.4 GHz: freq_reg = freq_hz / 198.3642578125
    uint32_t freq_reg = (uint32_t)((double)freq_hz / 198.3642578125);

    uint8_t cmd_data[3];
    cmd_data[0] = (freq_reg >> 16) & 0xFF;
    cmd_data[1] = (freq_reg >> 8) & 0xFF;
    cmd_data[2] = freq_reg & 0xFF;

    sband_write_command(SX1280_CMD_SET_RF_FREQUENCY, cmd_data, 3);
}

// Set modulation parameters for LoRa
void sband_set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr) {
    uint8_t cmd_data[3];
    cmd_data[0] = sf;  // Spreading factor
    cmd_data[1] = bw;  // Bandwidth
    cmd_data[2] = cr;  // Coding rate

    sband_write_command(SX1280_CMD_SET_MODULATION_PARAMS, cmd_data, 3);
}

// Set packet parameters
void sband_set_packet_params(uint8_t preamble_len, uint8_t header_type,
                              uint8_t payload_len, uint8_t crc, uint8_t iq) {
    uint8_t cmd_data[7];
    cmd_data[0] = preamble_len;  // Preamble length
    cmd_data[1] = header_type;   // Header type (0x00 = variable, 0x80 = fixed)
    cmd_data[2] = payload_len;   // Payload length
    cmd_data[3] = crc;           // CRC (0x20 = on, 0x00 = off)
    cmd_data[4] = iq;            // IQ (0x40 = standard, 0x00 = inverted)
    cmd_data[5] = 0x00;          // Unused
    cmd_data[6] = 0x00;          // Unused

    sband_write_command(SX1280_CMD_SET_PACKET_PARAMS, cmd_data, 7);
}

// Set buffer base address (TX and RX base pointers in 256-byte buffer)
void sband_set_buffer_base_address(uint8_t tx_base, uint8_t rx_base) {
    uint8_t cmd_data[2];
    cmd_data[0] = tx_base;  // TX base address in buffer (typically 0x00)
    cmd_data[1] = rx_base;  // RX base address in buffer (typically 0x00)

    sband_write_command(SX1280_CMD_SET_BUFFER_BASE_ADDRESS, cmd_data, 2);
}

// Set TX parameters
void sband_set_tx_params(int8_t power, uint8_t ramp_time) {
    // SX1280 power range: -18 to +13 dBm
    // Power byte: 0x00 = -18 dBm, 0x1F = +13 dBm
    uint8_t power_byte = (power + 18) & 0x1F;

    uint8_t cmd_data[2];
    cmd_data[0] = power_byte;  // TX power
    cmd_data[1] = ramp_time;   // Ramp time (0x02 = 20us, 0x04 = 40us, etc.)

    sband_write_command(SX1280_CMD_SET_TX_PARAMS, cmd_data, 2);
}

// Set DIO IRQ parameters
void sband_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask,
                                uint16_t dio2_mask, uint16_t dio3_mask) {
    uint8_t cmd_data[8];
    cmd_data[0] = (irq_mask >> 8) & 0xFF;
    cmd_data[1] = irq_mask & 0xFF;
    cmd_data[2] = (dio1_mask >> 8) & 0xFF;
    cmd_data[3] = dio1_mask & 0xFF;
    cmd_data[4] = (dio2_mask >> 8) & 0xFF;
    cmd_data[5] = dio2_mask & 0xFF;
    cmd_data[6] = (dio3_mask >> 8) & 0xFF;
    cmd_data[7] = dio3_mask & 0xFF;

    sband_write_command(SX1280_CMD_SET_DIO_IRQ_PARAMS, cmd_data, 8);
}

// Get IRQ status
uint16_t sband_get_irq_status(void) {
    uint8_t status[2];
    sband_read_command(SX1280_CMD_GET_IRQ_STATUS, status, 2);
    return ((uint16_t)status[0] << 8) | status[1];
}

// Clear IRQ status
void sband_clear_irq_status(uint16_t mask) {
    uint8_t cmd_data[2];
    cmd_data[0] = (mask >> 8) & 0xFF;
    cmd_data[1] = mask & 0xFF;

    sband_write_command(SX1280_CMD_CLEAR_IRQ_STATUS, cmd_data, 2);
}

// Put radio in RX mode (listen)
void sband_listen(void) {
    // Update the antenna relay
    gpio_put(sband_txen_pin, 0);  // Disable TX path
    gpio_put(sband_rxen_pin, 1);  // Enable RX path
    uint8_t cmd_data[3];
    cmd_data[0] = 0x00;  // periodBase
    cmd_data[1] = 0xFF;  // periodBaseCount MSB (0xFFFF = continuous RX)
    cmd_data[2] = 0xFF;  // periodBaseCount LSB

    sband_write_command(SX1280_CMD_SET_RX, cmd_data, 3);

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] WARNING: Returning from set mode despite BUSY timeout\n",
               __FILE__, __func__, __LINE__);
    }
}

// Put radio in TX mode (transmit)
void sband_transmit(void) {
    // Update the antenna relay
    gpio_put(sband_rxen_pin, 0);  // Disable RX path
    gpio_put(sband_txen_pin, 1);  // Enable TX path
    uint8_t cmd_data[3];
    cmd_data[0] = 0x00;  // periodBase
    cmd_data[1] = 0x00;  // periodBaseCount MSB (0x0000 = no timeout)
    cmd_data[2] = 0x00;  // periodBaseCount LSB

    sband_write_command(SX1280_CMD_SET_TX, cmd_data, 3);

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] WARNING: Returning from set mode despite BUSY timeout\n",
               __FILE__, __func__, __LINE__);
    }
}

// Check if TX is done
uint8_t sband_tx_done(void) {
    uint16_t irq_status = sband_get_irq_status();
    if (irq_status & SX1280_IRQ_TX_DONE) {
        sband_clear_irq_status(SX1280_IRQ_TX_DONE);
        return 1;
    }
    return 0;
}

// Check if RX is done
uint8_t sband_rx_done(void) {
    uint16_t irq_status = sband_get_irq_status();
    return (irq_status & SX1280_IRQ_RX_DONE) ? 1 : 0;
}

// Write packet to FIFO
void sband_packet_to_fifo(uint8_t *buf, uint8_t n) {
    uint8_t cmd_data[258];
    cmd_data[0] = 0x00;  // Offset in buffer
    memcpy(cmd_data + 1, buf, n);

    sband_write_command(SX1280_CMD_WRITE_BUFFER, cmd_data, n + 1);
}

// Read packet from FIFO
// Returns the number of bytes read
uint8_t sband_packet_from_fifo(uint8_t *buf) {
    // Get RX buffer status first
    uint8_t status[2];
    sband_read_command(SX1280_CMD_GET_RX_BUFFER_STATUS, status, 2);

    uint8_t payload_len = status[0];
    uint8_t offset = status[1];

    if (payload_len > 0 && payload_len <= 256) {
        // Read the buffer
        sband_tx_combined[0] = SX1280_CMD_READ_BUFFER;
        sband_tx_combined[1] = offset;
        memset(sband_tx_combined + 2, 0x00, payload_len + 1);  // NOP + dummy bytes

        sband_spi_transfer(sband_tx_combined, sband_rx_combined, payload_len + 3);

        // Copy data (skip status + NOP + offset)
        memcpy(buf, sband_rx_combined + 3, payload_len);

        return payload_len;
    }

    return 0;
}

// Get SNR (in dB)
int8_t sband_get_snr(void) {
    uint8_t status[5];
    sband_read_command(SX1280_CMD_GET_PACKET_STATUS, status, 5);

    // For LoRa: byte 1 is SNR (signed, in 0.25 dB steps)
    int8_t snr_raw = (int8_t)status[1];
    return snr_raw / 4;  // Convert to dB
}

// Get RSSI (in dBm)
int16_t sband_get_rssi(void) {
    uint8_t status[5];
    sband_read_command(SX1280_CMD_GET_PACKET_STATUS, status, 5);

    // For LoRa: byte 0 is RSSI
    // RSSI = -value/2 (dBm)
    int16_t rssi = -(int16_t)status[0] / 2;
    return rssi;
}

// Get current operating mode (reads from chip via CMD_GET_STATUS)
sx1280_mode_t sband_get_mode(void) {
    uint8_t status;
    sband_read_command(SX1280_CMD_GET_STATUS, &status, 1);

    // Extract mode from bits [7:5] (datasheet Table 11-6)
    uint8_t mode_bits = (status >> 5) & 0x07;

    // Map hardware status bits to mode enum
    switch (mode_bits) {
        case 0b010: return SX1280_MODE_STDBY_RC;
        case 0b011: return SX1280_MODE_STDBY_XOSC;
        case 0b100: return SX1280_MODE_FS;
        case 0b101: return SX1280_MODE_RX;
        case 0b110: return SX1280_MODE_TX;
        default:    return SX1280_MODE_SLEEP;
    }
}

// Get packet type (reads from chip via CMD_GET_PACKET_TYPE)
sx1280_packet_type_t sband_get_packet_type(void) {
    uint8_t packet_type;
    sband_read_command(SX1280_CMD_GET_PACKET_TYPE, &packet_type, 1);
    return (sx1280_packet_type_t)packet_type;
}

// Verify SX1280 chip is present by reading version string
// Returns 1 on success (chip responding), 0 on failure
int sband_verify_chip(char *version_out) {
    char version[16] = {0};

    // Read 16-byte version string from register 0x01F0
    sband_read_register(SX1280_REG_VERSION_STRING, (uint8_t*)version, 16);

    // Check if chip is responding (not all 0x00 or all 0xFF)
    int all_zero = 1, all_ff = 1;
    for (int i = 0; i < 16; i++) {
        if (version[i] != 0x00) all_zero = 0;
        if (version[i] != 0xFF) all_ff = 0;
    }

    if (all_zero || all_ff) {
        return 0;  // Chip not responding
    }

    // Copy version string to output buffer if provided
    if (version_out) {
        memcpy(version_out, version, 16);
    }

    return 1;  // Success
}

// Decode and print SX1280 status byte
static void sband_print_status_byte(uint8_t status, const char *context) {
    uint8_t circuit_mode = (status >> 5) & 0x07;
    uint8_t cmd_status = (status >> 2) & 0x07;
    uint8_t busy = status & 0x01;

    printf("SBand Status [%s]: 0x%02X\n", context, status);
    
    // Decode circuit mode (bits 7:5)
    printf("  Circuit Mode (bits 7:5): 0x%X = ", circuit_mode);
    switch (circuit_mode) {
        case 0x0: printf("Reserved\n"); break;
        case 0x1: printf("Reserved\n"); break;
        case 0x2: printf("STDBY_RC\n"); break;
        case 0x3: printf("STDBY_XOSC\n"); break;
        case 0x4: printf("FS\n"); break;
        case 0x5: printf("RX\n"); break;
        case 0x6: printf("TX\n"); break;
        case 0x7: printf("Reserved\n"); break;
    }
    
    // Decode command status (bits 4:2)
    printf("  Command Status (bits 4:2): 0x%X = ", cmd_status);
    switch (cmd_status) {
        case 0x0: printf("Reserved\n"); break;
        case 0x1: printf("Success (command processed)\n"); break;
        case 0x2: printf("Data available to host\n"); break;
        case 0x3: printf("Command timeout (watchdog)\n"); break;
        case 0x4: printf("Command processing error (invalid opcode/params)\n"); break;
        case 0x5: printf("Failure to execute command\n"); break;
        case 0x6: printf("Command TX done\n"); break;
        case 0x7: printf("Reserved\n"); break;
    }
    
    // Decode busy bit (bit 0)
    printf("  Busy (bit 0): %d = %s\n", busy, busy ? "BUSY (processing)" : "IDLE");
}

// Initialize SBand radio
int sband_init(void) {
    sband_spi = SAMWISE_SBAND_SPI_INSTANCE;
    sband_cs_pin = SAMWISE_SBAND_CS_PIN;
    sband_rst_pin = SAMWISE_SBAND_RST_PIN;
    sband_d0_pin = SAMWISE_SBAND_D0_PIN;

    // Configure GPIO pins
    gpio_init(sband_rst_pin);
    gpio_set_dir(sband_rst_pin, GPIO_OUT);
    gpio_put(sband_rst_pin, 1);

    gpio_init(sband_cs_pin);
    gpio_set_dir(sband_cs_pin, GPIO_OUT);
    gpio_put(sband_cs_pin, 1);

    gpio_init(sband_d0_pin);
    gpio_set_dir(sband_d0_pin, GPIO_IN);
    gpio_pull_down(sband_d0_pin);

    // Initialize RX enable pin (driven high for RX, low for TX)
    sband_rxen_pin = SAMWISE_SBAND_RXEN_PIN;
    gpio_init(sband_rxen_pin);
    gpio_set_dir(sband_rxen_pin, GPIO_OUT);
    gpio_put(sband_rxen_pin, 0); // Default to TX disabled (low)

    // Initialize TX enable pin (driven high for TX, low for RX)
    sband_txen_pin = SAMWISE_SBAND_TXEN_PIN;
    gpio_init(sband_txen_pin);
    gpio_set_dir(sband_txen_pin, GPIO_OUT);
    gpio_put(sband_txen_pin, 0); // Default to TX disabled (low)

    // Initialize D1 pin for interrupt (DIO1)
    gpio_init(SAMWISE_SBAND_D1_PIN);
    gpio_set_dir(SAMWISE_SBAND_D1_PIN, GPIO_IN);
    gpio_pull_down(SAMWISE_SBAND_D1_PIN);

    // Initialize SPI1 for SBand (separate bus from UHF's SPI0)
    gpio_set_function(SAMWISE_SBAND_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SAMWISE_SBAND_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SAMWISE_SBAND_MISO_PIN, GPIO_FUNC_SPI);

    // Initialize SPI1 at 5 MHz
    // SX1280 supports CPOL = 0, CPHA = 0 (mode 0), MSB first
    spi_init(sband_spi, SX1280_SPI_BAUDRATE);
    spi_set_format(sband_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Claim DMA channels for SPI transfers
    sband_tx_dma_chan = dma_claim_unused_channel(true);
    sband_rx_dma_chan = dma_claim_unused_channel(true);

    // Configure RX DMA (same for all transactions)
    dma_channel_config rx_cfg = dma_channel_get_default_config(sband_rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&rx_cfg, spi_get_dreq(sband_spi, false));
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    dma_channel_configure(sband_rx_dma_chan, &rx_cfg, NULL, &spi_get_hw(sband_spi)->dr,
                         0, false);

    // Reset radio
    if (!sband_reset()) {
        printf("SBand: ERROR: Reset failed with BUSY timeout - aborting initialization\n");
        // Release DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] ERROR: BUSY timeout - aborting initialization\n",
               __FILE__, __func__, __LINE__);
        // Release DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    // Send GET_STATUS to verify chip is responsive (simple 1-byte command)
    uint8_t cmd_buf[1] = {SX1280_CMD_GET_STATUS};
    uint8_t status_buf[1];
    sband_spi_transfer(cmd_buf, status_buf, 1);
    printf("SBand: Status after reset: 0x%02X\n", status_buf[0]);
// Add detailed status printing
    sband_print_status_byte(status_buf[0], "After reset");
    if (status_buf[0] == 0x00 || status_buf[0] == 0xFF) {
        printf("SBand: ERROR: No status byte from SX1280 after reset\n");
        // Release DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    // Chip should already be in STANDBY_RC after reset, but set it explicitly
    sband_set_mode(SX1280_MODE_STDBY_RC);

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] ERROR: BUSY timeout - aborting initialization\n",
               __FILE__, __func__, __LINE__);
        // Release DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    // Configure for LoRa mode
    sband_set_packet_type(SX1280_PACKET_TYPE_LORA);

    if (sband_still_busy_after_wait()) {
        printf("[File: %s, Function: %s, Line: %d] ERROR: BUSY timeout - aborting initialization\n",
            __FILE__, __func__, __LINE__);
        // Release DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    sband_set_rf_frequency(2427000000);  // 2427 MHz
    sband_set_modulation_params(SX1280_LORA_SF5, SX1280_LORA_BW_200, SX1280_LORA_CR_4_5);
    sband_set_packet_params(12, 0x00, 253, 0x20, 0x40);  // Preamble=12, variable header, 253 bytes, CRC on, IQ normal
    sband_set_buffer_base_address(0x00, 0x00);  // TX and RX both start at 0x00 in 256-byte buffer
    sband_set_tx_params(13, 0x02);  // 13 dBm, ramp 20us

    // Configure interrupts
    uint16_t irq_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR;
    sband_set_dio_irq_params(irq_mask, irq_mask, 0, 0);

    printf("SBand: SX1280 DMA initialized (DMA TX=%d, RX=%d)\n",
           sband_tx_dma_chan, sband_rx_dma_chan);

    // Verify chip is present
    char version[16];
    if (!sband_verify_chip(version)) {
        printf("SBand: ERROR: SX1280 not found (no response from chip)\n");

        // release the two DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);
        return -1;
    }

    // Print version string for debugging
    printf("SBand: SX1280 detected, version: ");
    for (int i = 0; i < 16; i++) {
        if (version[i] >= 32 && version[i] <= 126) {
            printf("%c", version[i]);
        } else {
            printf("\\x%02X", (uint8_t)version[i]);
        }
    }
    printf("\n");

    return 0;
}

// JSON configuration loading (simple string parsing)
int sband_load_config_json(const char *json_string, sband_config_t *config) {
    if (json_string == NULL || config == NULL) {
        return -1;
    }

    // Simple string parsing for the specific format
    char *mode_start = strstr(json_string, "\"Mode\": \"");
    if (mode_start) {
        sscanf(mode_start, "\"Mode\": \"%15[^\"]\"", config->mode);
    }

    char *freq_start = strstr(json_string, "\"Freq\": ");
    if (freq_start) {
        sscanf(freq_start, "\"Freq\": %hu", &config->freq_mhz);
    }

    char *power_start = strstr(json_string, "\"Power\": ");
    if (power_start) {
        sscanf(power_start, "\"Power\": %hhu", &config->power);
    }

    char *bw_start = strstr(json_string, "\"BW\": ");
    if (bw_start) {
        sscanf(bw_start, "\"BW\": %hu", &config->bw_khz);
    }

    char *sf_start = strstr(json_string, "\"SF\": ");
    if (sf_start) {
        sscanf(sf_start, "\"SF\": %hhu", &config->sf);
    }

    char *cr_start = strstr(json_string, "\"CR\": ");
    if (cr_start) {
        sscanf(cr_start, "\"CR\": %hhu", &config->cr);
    }

    return 0;
}
