#pragma once
//#include "macros.h"
#include "hardware/spi.h" // Include the Pico SDK SPI header for spi_t
#include <stdint.h>       // Include standard integer types
#include "bit-support.h"  // Include bit manipulation functions

#define RFM96_SPI_BAUDRATE (5 * 1000 * 1000)

#if defined (PICO)
    /*
    * Pins for only the pico here...
    * CMakeLists.txt says add_compile_definitions(PICO)
    */

    #define SAMWISE_RF_SPI (0)
    #define SAMWISE_RF_SCK_PIN (18)
    #define SAMWISE_RF_MOSI_PIN (19)
    #define SAMWISE_RF_MISO_PIN (16)
    #define SAMWISE_RF_RST_PIN (21)
    #define SAMWISE_RF_CS_PIN (17)
    #define SAMWISE_RF_D0_PIN (20)

    #define SAMWISE_MPPT_I2C (I2C_INSTANCE(0))
    #define SAMWISE_MPPT_SDA_PIN (12)
    #define SAMWISE_MPPT_SCL_PIN (13)

    #define SAMWISE_WATCHDOG_FEED_PIN (6)

#else

    /*
    * Pins on the PiCubed
    * IMPORTANT: Must be updated to keep in sync with the schematic/avionics!
    * CMakeLists.txt has add_compile_definitions(PICO) commented out
    */
    #define SAMWISE_NEOPIXEL_PIN (0)

    #define SAMWSIE_MPPT_SHDN_2_PIN (1)
    #define SAMWISE_MPPT_STAT_2_PIN (2)

    #define SAMWISE_BAT_HEATER_PIN (3)

    #define SAMWISE_MPPT_I2C (I2C_INSTANCE(0))
    #define SAMWISE_MPPT_SDA_PIN (4)
    #define SAMWISE_MPPT_SCL_PIN (5)

    #define SAMWISE_POWER_MONITOR_I2C (I2C_INSTANCE(1))
    #define SAMWISE_POWER_MONITOR_SDA_PIN (38)
    #define SAMWISE_POWER_MONITOR_SCL_PIN (39)

    #define SAMWISE_WATCHDOG_FEED_PIN (6)

    #define SAMWISE_MPPT_STAT_1_PIN (7)
    #define SAMWISE_MPPT_SHDN_1_PIN (8)

    #define SAMWISE_SIDE_DEPLOY_DETECT_B_PIN (9)
    #define SAMWISE_SIDE_DEPLOY_DETECT_A_PIN (10)

    #define SAMWISE_RF_SPI (1)
    #define SAMWISE_RF_REGULATOR_PIN (21)
    #define SAMWISE_RF_RST_PIN (11)
    #define SAMWISE_RF_MISO_PIN (12)
    #define SAMWISE_RF_CS_PIN (13)
    #define SAMWISE_RF_SCK_PIN (14)
    #define SAMWISE_RF_MOSI_PIN (15)
    #define SAMWISE_RF_D0_PIN (20)

    #define SAMWISE_WATCHDOG_FEED_PIN (6)

#endif

// Name a datatype for the SPI pins which makes it easier to create the storage and pass it around
typedef struct spi_pins_t
{
    uint8_t CIPO;
    uint8_t COPI;
    uint8_t SCK;
    uint8_t CS;
    uint8_t RESET;
    uint8_t D0;
    spi_inst_t *spi;
} spi_pins_t;

typedef struct rfm96_args_t
{
    uint16_t Freq;
    uint16_t BW;
    uint16_t SF;
    uint16_t CR;
    uint16_t CRCon;
    uint16_t LDRO;   // Low Data Rate Optimize
    uint16_t LNAon;
    uint16_t PAoff;
    uint16_t TXpower;
    uint16_t high_power;
} rfm96_args_t;



typedef enum
{
    SLEEP_MODE = 0,
    STANDBY_MODE = 1,
    FS_TX_MODE = 2,
    TX_MODE = 3,
    FS_RX_MODE = 4,
    RX_MODE = 5,
} rfm96_mode_t;

/**
 * Assert a certain condition at runtime and raise an error if it is false.
 */
#define ASSERT(condition)                                                                                                                                               \
    {                                                \
        if (!(condition))                            \
        {                                            \
            printf("Assertion failed: " #condition); \
        }                                            \
    }
/*
 * Initializes an RFM9X radio.
 */
int rfm96_init(spi_pins_t *spi_pins);

#define _SAP_FLAGS_ACK_REQUEST 2

/*
 * Returns the chip version from the RFM9X
 */
uint32_t rfm96_version(spi_pins_t spi_pins);

/*
 * Send a raw transmission from the RFM9X.
 *
 * r: the radio
 * data: the data to send
 * l: the length of the data. Must be less than `PAYLOAD_SIZE`
 * keep_listening: 0 to stop listening after sending, 1 to keep blocking
 * destination: radio to send it to. 255 is broadcast.
 * node: our address
 * identifier: Sequence number — if sending multiple packets, increment by one
 * per packet.
 * flags:
 */

uint32_t rfm96_version(spi_pins_t spi_pins);

void rfm96_listen();
void rfm96_transmit();

uint8_t rfm96_tx_done();
uint8_t rfm96_rx_done();

uint8_t rfm96_packet_to_fifo(uint8_t *buf, uint8_t n);
uint8_t rfm96_packet_from_fifo(uint8_t *buf);
void rfm96_set_tx_power(int8_t power);
void rfm96_set_mode(rfm96_mode_t mode);
uint8_t rfm96_crc_error();

typedef enum
{
    _RH_RF95_REG_00_FIFO = 0x00,
    _RH_RF95_REG_01_OP_MODE = 0x01,
    _RH_RF95_REG_06_FRF_MSB = 0x06,
    _RH_RF95_REG_07_FRF_MID = 0x07,
    _RH_RF95_REG_08_FRF_LSB = 0x08,
    _RH_RF95_REG_09_PA_CONFIG = 0x09,
    _RH_RF95_REG_0A_PA_RAMP = 0x0A,
    _RH_RF95_REG_0B_OCP = 0x0B,
    _RH_RF95_REG_0C_LNA = 0x0C,
    _RH_RF95_REG_0D_FIFO_ADDR_PTR = 0x0D,
    _RH_RF95_REG_0E_FIFO_TX_BASE_ADDR = 0x0E,
    _RH_RF95_REG_0F_FIFO_RX_BASE_ADDR = 0x0F,
    _RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR = 0x10,
    _RH_RF95_REG_11_IRQ_FLAGS_MASK = 0x11,
    _RH_RF95_REG_12_IRQ_FLAGS = 0x12,
    _RH_RF95_REG_13_RX_NB_BYTES = 0x13,
    _RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB = 0x14,
    _RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB = 0x15,
    _RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB = 0x16,
    _RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB = 0x17,
    _RH_RF95_REG_18_MODEM_STAT = 0x18,
    _RH_RF95_REG_19_PKT_SNR_VALUE = 0x19,
    _RH_RF95_REG_1A_PKT_RSSI_VALUE = 0x1A,
    _RH_RF95_REG_1B_RSSI_VALUE = 0x1B,
    _RH_RF95_REG_1C_HOP_CHANNEL = 0x1C,
    _RH_RF95_REG_1D_MODEM_CONFIG1 = 0x1D,
    _RH_RF95_REG_1E_MODEM_CONFIG2 = 0x1E,
    _RH_RF95_REG_1F_SYMB_TIMEOUT_LSB = 0x1F,
    _RH_RF95_REG_20_PREAMBLE_MSB = 0x20,
    _RH_RF95_REG_21_PREAMBLE_LSB = 0x21,
    _RH_RF95_REG_22_PAYLOAD_LENGTH = 0x22,
    _RH_RF95_REG_23_MAX_PAYLOAD_LENGTH = 0x23,
    _RH_RF95_REG_24_HOP_PERIOD = 0x24,
    _RH_RF95_REG_25_FIFO_RX_BYTE_ADDR = 0x25,
    _RH_RF95_REG_26_MODEM_CONFIG3 = 0x26,

    /**
     * In this register:
     * Bits 7-6: Dio0Mapping
     * Bits 5-4: Dio1Mapping
     * Bits 3-2: Dio2Mapping
     * Bits 1-0: Dio3Mapping
     *
     * (p. 100 of documentation)
     */
    _RH_RF95_REG_40_DIO_MAPPING1 = 0x40,
    _RH_RF95_REG_41_DIO_MAPPING2 = 0x41,
    _RH_RF95_REG_42_VERSION = 0x42,

    _RH_RF95_REG_4B_TCXO = 0x4B,
    _RH_RF95_REG_4D_PA_DAC = 0x4D,
    _RH_RF95_REG_5B_FORMER_TEMP = 0x5B,
    _RH_RF95_REG_61_AGC_REF = 0x61,
    _RH_RF95_REG_62_AGC_THRESH1 = 0x62,
    _RH_RF95_REG_63_AGC_THRESH2 = 0x63,
    _RH_RF95_REG_64_AGC_THRESH3 = 0x64,

    _RH_RF95_DETECTION_OPTIMIZE = 0x31,
    _RH_RF95_DETECTION_THRESHOLD = 0x37,

    _RH_RF95_PA_DAC_DISABLE = 0x04,
    _RH_RF95_PA_DAC_ENABLE = 0x07,

    // The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
    _RH_RF95_FSTEP = 32000000 / 524288,

    // RadioHead specific compatibility constants.
    _RH_BROADCAST_ADDRESS = 0xFF,

    // The acknowledgement bit in the FLAGS
    // The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits
    // are reserved for application layer use.
    _RH_FLAGS_ACK = 0x80,
    _RH_FLAGS_RETRY = 0x40,
} rfm96_reg_t;