#pragma once
#include <stdint.h>
#include "hardware/spi.h"
#include "pins.h"

// Configuration structure (from JSON)
typedef struct {
    char mode[16];      // "lora" or "flrc"
    uint16_t freq_mhz;  // 2427 MHz
    uint8_t power;      // 30 dBm (actually -18 to +13 dBm range)
    uint16_t bw_khz;    // 200 kHz
    uint8_t sf;         // 5
    uint8_t cr;         // 5 (CR 4/5)
} sband_config_t;

// Operating modes
typedef enum {
    SX1280_MODE_SLEEP = 0x00,
    SX1280_MODE_STDBY_RC = 0x01,
    SX1280_MODE_STDBY_XOSC = 0x02,
    SX1280_MODE_FS = 0x03,
    SX1280_MODE_TX = 0x04,
    SX1280_MODE_RX = 0x05
} sx1280_mode_t;

// Packet types
typedef enum {
    SX1280_PACKET_TYPE_GFSK = 0x00,
    SX1280_PACKET_TYPE_LORA = 0x01,
    SX1280_PACKET_TYPE_RANGING = 0x02,
    SX1280_PACKET_TYPE_FLRC = 0x03,
    SX1280_PACKET_TYPE_BLE = 0x04
} sx1280_packet_type_t;

// LoRa bandwidth values
typedef enum {
    SX1280_LORA_BW_200 = 0x34,
    SX1280_LORA_BW_400 = 0x26,
    SX1280_LORA_BW_800 = 0x18,
    SX1280_LORA_BW_1600 = 0x0A
} sx1280_lora_bw_t;

// LoRa spreading factors
typedef enum {
    SX1280_LORA_SF5 = 0x50,
    SX1280_LORA_SF6 = 0x60,
    SX1280_LORA_SF7 = 0x70,
    SX1280_LORA_SF8 = 0x80,
    SX1280_LORA_SF9 = 0x90,
    SX1280_LORA_SF10 = 0xA0,
    SX1280_LORA_SF11 = 0xB0,
    SX1280_LORA_SF12 = 0xC0
} sx1280_lora_sf_t;

// LoRa coding rates
typedef enum {
    SX1280_LORA_CR_4_5 = 0x01,
    SX1280_LORA_CR_4_6 = 0x02,
    SX1280_LORA_CR_4_7 = 0x03,
    SX1280_LORA_CR_4_8 = 0x04,
    SX1280_LORA_CR_LI_4_5 = 0x05,
    SX1280_LORA_CR_LI_4_6 = 0x06,
    SX1280_LORA_CR_LI_4_7 = 0x07,
    SX1280_LORA_CR_LI_4_8 = 0x08
} sx1280_lora_cr_t;

// IRQ masks
#define SX1280_IRQ_TX_DONE           0x0001
#define SX1280_IRQ_RX_DONE           0x0002
#define SX1280_IRQ_SYNC_WORD_VALID   0x0004
#define SX1280_IRQ_SYNC_WORD_ERROR   0x0008
#define SX1280_IRQ_HEADER_VALID      0x0010
#define SX1280_IRQ_HEADER_ERROR      0x0020
#define SX1280_IRQ_CRC_ERROR         0x0040
#define SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE 0x0080
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED 0x0100
#define SX1280_IRQ_RANGING_MASTER_RESULT_VALID 0x0200
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT 0x0400
#define SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID 0x0800
#define SX1280_IRQ_CAD_DONE          0x1000
#define SX1280_IRQ_CAD_DETECTED      0x2000
#define SX1280_IRQ_RX_TX_TIMEOUT     0x4000
#define SX1280_IRQ_PREAMBLE_DETECTED 0x8000

// API functions
int sband_init(spi_pins_t *spi_pins);
void sband_reset(void);
void sband_set_mode(sx1280_mode_t mode);
void sband_set_packet_type(sx1280_packet_type_t type);
void sband_set_rf_frequency(uint32_t freq_hz);
void sband_set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr);
void sband_set_packet_params(uint8_t preamble_len, uint8_t header_type, uint8_t payload_len, uint8_t crc, uint8_t iq);
void sband_set_buffer_base_address(uint8_t tx_base, uint8_t rx_base);
void sband_set_tx_params(int8_t power, uint8_t ramp_time);
void sband_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
void sband_listen(void);
void sband_transmit(void);
uint8_t sband_tx_done(void);
uint8_t sband_rx_done(void);
void sband_packet_to_fifo(uint8_t *buf, uint8_t n);
uint8_t sband_packet_from_fifo(uint8_t *buf);
int8_t sband_get_snr(void);
int16_t sband_get_rssi(void);
uint16_t sband_get_irq_status(void);
void sband_clear_irq_status(uint16_t mask);

// BUSY timeout error counter (exported for main.c error display)
extern volatile uint32_t sband_busy_timeout_count;

// Chip verification (reads 16-byte version string from register 0x01F0)
int sband_verify_chip(char *version_out);

// Getter functions (only for chip-readable values)
sx1280_mode_t sband_get_mode(void);
sx1280_packet_type_t sband_get_packet_type(void);

// JSON configuration loading
int sband_load_config_json(const char *json_string, sband_config_t *config);  //PHM is this necessary?
