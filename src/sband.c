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
 * - No BUSY pin - use timed delays (15us after reset, 1ms after mode change)
 * - IRQ status is 16-bit (vs 8-bit)
 * - Frequency range 2.4 GHz (vs 437 MHz)
 * - Shared SPI bus with UHF radio
 */

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
#define SX1280_CMD_GET_IRQ_STATUS            0x15
#define SX1280_CMD_CLEAR_IRQ_STATUS          0x97
#define SX1280_CMD_GET_RX_BUFFER_STATUS      0x17
#define SX1280_CMD_GET_PACKET_STATUS         0x1D
#define SX1280_CMD_GET_RSSI_INST             0x1F

#define SX1280_FIRMWARE_VERSION_MSB 0x0153       // Firmware version register. PHM why is this three nibbles?
#define SX1280_FIRMWARE_VERSION_EXPECTED 0xA907         // Expected firmware version 43447

// Timeout for DMA transfers in microseconds (20ms)
#define DMA_TIMEOUT_US 20000

// SX1280 SPI baudrate (can go up to 18 MHz, using 5 MHz for reliability)
#define SX1280_SPI_BAUDRATE (5 * 1000 * 1000)

// Global SPI state
static spi_inst_t *sband_spi = NULL;
static uint8_t sband_cs_pin;
static uint8_t sband_rst_pin;
static uint8_t sband_d0_pin;
static int sband_tx_dma_chan = -1;
static int sband_rx_dma_chan = -1;

// DMA buffers
static uint8_t sband_tx_combined[258];
static uint8_t sband_rx_combined[258];

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

// Low-level SPI transfer with DMA
static void sband_spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    sband_timeout_t timeout;

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

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

// Reset the SX1280
void sband_reset(void) {
    gpio_put(sband_rst_pin, 0);
    sleep_us(50);
    gpio_put(sband_rst_pin, 1);
    sleep_us(100);  // Wait for chip to boot (15us min)
}

// Set operating mode
void sband_set_mode(sx1280_mode_t mode) {
    uint8_t cmd_data[1] = {mode};
    sband_write_command(SX1280_CMD_SET_STANDBY, cmd_data, 1);
    sleep_ms(1);  // Wait for mode change
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
    uint8_t cmd_data[3];
    cmd_data[0] = 0x02;  // Periodbase (not used in single mode)
    cmd_data[1] = 0xFF;  // Timeout MSB (0xFFFF = continuous)
    cmd_data[2] = 0xFF;  // Timeout LSB

    sband_write_command(SX1280_CMD_SET_RX, cmd_data, 3);
}

// Put radio in TX mode (transmit)
void sband_transmit(void) {
    uint8_t cmd_data[3];
    cmd_data[0] = 0x02;  // Periodbase
    cmd_data[1] = 0xFF;  // Timeout MSB (no timeout)
    cmd_data[2] = 0xFF;  // Timeout LSB

    sband_write_command(SX1280_CMD_SET_TX, cmd_data, 3);
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

uint16_t sband_chip_id(void) {
    uint8_t chip_id_bytes[2];
    sband_read_command(SX1280_FIRMWARE_VERSION_MSB, chip_id_bytes, 2);
    return ((uint16_t)chip_id_bytes[0] << 8) | chip_id_bytes[1];
}

// Initialize SBand radio
int sband_init(spi_pins_t *spi_pins) {
    sband_spi = spi_pins->spi;
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
    sband_reset();

    // Set standby mode
    sband_set_mode(SX1280_MODE_STDBY_RC);

    // Configure for LoRa mode
    sband_set_packet_type(SX1280_PACKET_TYPE_LORA);
    sband_set_rf_frequency(2427000000);  // 2427 MHz
    sband_set_modulation_params(SX1280_LORA_SF5, SX1280_LORA_BW_200, SX1280_LORA_CR_4_5);
    sband_set_packet_params(12, 0x00, 253, 0x20, 0x40);  // Preamble=12, variable header, 253 bytes, CRC on, IQ normal
    sband_set_tx_params(13, 0x02);  // 13 dBm, ramp 20us

    // Configure interrupts
    uint16_t irq_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR;
    sband_set_dio_irq_params(irq_mask, irq_mask, 0, 0);

    printf("SBand: SX1280 DMA initialized (DMA TX=%d, RX=%d)\n",
           sband_tx_dma_chan, sband_rx_dma_chan);

    uint8_t chip_id = sband_chip_id();
    if (chip_id != SX1280_FIRMWARE_VERSION_EXPECTED) {
        printf("SBand: ERROR: Unexpected chip ID: 0x%02X, expected: 0x%02X\n", chip_id, SX1280_FIRMWARE_VERSION_EXPECTED);

        // release the two DMA channels before returning error
        dma_channel_unclaim(sband_tx_dma_chan);
        dma_channel_unclaim(sband_rx_dma_chan);        
        return -1;
    }

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
