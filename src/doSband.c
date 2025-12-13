#include "doSband.h"
#include "sband.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "ws2812.h"  // For LED control (red, white)
#include "main_gps_uart_shared_buffer.h"  // For BUFLEN definition

/*  

Called by main.c and calling sband.c, this function implements SBand superloop 
functionality for PiTest.

RX is DIO0 ISR driven, using a packet queue to fill a histogram of transmitted power levels.
Non-blocking TX is implemented sends one packet at new power level each invocation.
LoRa Parameters are hard coded.
Lively LED color indication of received packets and transmissions.

*/


// Packet queue structure definitions (for ISR access)
#define QUEUE_SIZE_SBAND 15
#define PACKET_SIZE_SBAND 256
typedef struct {
    uint8_t data[PACKET_SIZE_SBAND];
    uint8_t length;
    int8_t snr;
    int16_t rssi;
} packet_sband_t;

// Global queue variables for ISR access
volatile packet_sband_t packet_queue_sband[QUEUE_SIZE_SBAND];
volatile uint8_t queue_head_sband = 0;  // ISR writes here
volatile uint8_t queue_tail_sband = 0;  // Main loop reads here
volatile uint8_t queue_count_sband = 0;
volatile absolute_time_t last_packet_time_sband;  // For LED green timing

// Global CRC error counter (similar to UHF)
volatile uint8_t nCRC_sband = 0;

// Power histogram for received power levels
static int power_histogram_sband[32] = {0};  // histogram for -18 to +13 dBm range

// Timing variables
static absolute_time_t previous_time_Sband_RX;
static absolute_time_t previous_time_Sband_TX;
static uint32_t interval_Sband_RX = 1000000;        // 1 second
static uint32_t interval_Sband_TX = 20*1010*1000;   // 20.2 seconds

// State machine variables for non-blocking TX
static bool Sband_Transmitting = false;
static int current_tx_power_sband = 10;  // Start at 10 dBm, count down to -1
static bool sband_rx_active = false;      // Track if we received packets
static absolute_time_t sband_last_rx_time;
static int radio_initialized = -1;      //  sband radio initialized flag

// Static buffer for TX packets
static uint8_t tx_packet_sband[250];

// LED color contribution from SBand (for additive color mixing)
uint8_t sband_led_r = 0;
uint8_t sband_led_g = 0;
uint8_t sband_led_b = 0;

/*
 * DIO0 GPIO Interrupt Service Routine for SBand
 * Triggered when a packet is received (RX_DONE interrupt)
 * Copies packet from radio FIFO to queue
 */
void sband_dio0_isr(uint gpio, uint32_t events) {
    // Check IRQ status
    uint16_t irq_flags = sband_get_irq_status();

    if (irq_flags & SX1280_IRQ_RX_DONE) {
        // Check if queue has space
        if (queue_count_sband < QUEUE_SIZE_SBAND) {
            volatile packet_sband_t *pkt = &packet_queue_sband[queue_head_sband];

            // Read packet from FIFO
            pkt->length = sband_packet_from_fifo((uint8_t*)pkt->data);

            // Get SNR and RSSI
            pkt->snr = sband_get_snr();
            pkt->rssi = sband_get_rssi();

            // Update SNR and RSSI in packet starting at byte 25
            // Format: "SNR = %4d; RSSI = %4d"
            if (pkt->length > 25) {
                snprintf((char*)&pkt->data[25], 25, "SNR = %4d; RSSI = %4d",
                         (int)pkt->snr, (int)pkt->rssi);
            }

            // Update queue
            queue_head_sband = (queue_head_sband + 1) % QUEUE_SIZE_SBAND;
            queue_count_sband++;
            last_packet_time_sband = get_absolute_time();

            // Mark RX as active for LED color management
            sband_rx_active = true;
            sband_last_rx_time = get_absolute_time();
        }

        // Clear RX_DONE interrupt
        sband_clear_irq_status(SX1280_IRQ_RX_DONE);

        // Continue listening for next packet
        sband_listen();
    }

    if (irq_flags & SX1280_IRQ_CRC_ERROR) {
        nCRC_sband++;
        sband_clear_irq_status(SX1280_IRQ_CRC_ERROR);
    }
}

/*
 * Initialize SBand radio with tx_done test
 * Includes ISR setup and tx_done timing test
 */
void initSband(spi_pins_t *spi_pins) {
    // Initialize radio
    radio_initialized = sband_init(spi_pins);

    //For Hardware Debugging
    while (radio_initialized = sband_init(spi_pins) != PICO_OK) {
        printf("doSBand: Radio init failed, retrying in 0.1 second...\n");
        sleep_ms(100);
    }

    if (radio_initialized == 0) {
        printf("SBand: Radio initialized\n");

        // Disable ISR during test to prevent interference with tx_done()
        gpio_set_irq_enabled(SAMWISE_SBAND_D0_PIN, GPIO_IRQ_EDGE_RISE, false);

        // Check that we can know here when the transmitter has completed
        // by sending a short and a long packet and measuring the time it takes

        // Test 1: Short packet
        red();  // Indicate we are transmitting
        absolute_time_t start_time = get_absolute_time();

        // Send a short packet (5 bytes)
        uint8_t short_data[5] = {0xFF, 0xAA, 0x55, 0xBB, 0xCC};
        sband_packet_to_fifo(short_data, 5);
        sband_transmit();  // Send the packet

        int ip = 10000;
        while (!sband_tx_done() && ip--) { sleep_us(10); }
        if (!sband_tx_done()) printf("SBand: TX timed out (short)\n");
        printf("SBand: Time to send a short packet: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        // Test 2: Long packet
        start_time = get_absolute_time();

        // Send a long packet (250 bytes)
        uint8_t long_data[250];
        memset(long_data, 0xAA, 250);
        sband_packet_to_fifo(long_data, 250);
        sband_transmit();  // Send the packet

        ip = 100000;
        while (!sband_tx_done() && ip--) { sleep_us(10); }
        if (!sband_tx_done()) printf("SBand: TX timed out (long)\n");
        printf("SBand: Time to send a long packet: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        sband_listen(); // Set the radio to RX mode
        white();    // Indicate we are receiving

        // Enable ISR after test completes
        gpio_set_irq_enabled_with_callback(SAMWISE_SBAND_D0_PIN, GPIO_IRQ_EDGE_RISE, true, &sband_dio0_isr);
        printf("SBand: DIO0 ISR enabled on GPIO %d\n", SAMWISE_SBAND_D0_PIN);
    } else {
        printf("SBand: Radio initialization failed\n");
    }

    // Initialize timing variables
    previous_time_Sband_RX = get_absolute_time();   //  Baseline for checking the RX queue
    previous_time_Sband_TX = get_absolute_time();   //  Baseline for starting TX cycles
    last_packet_time_sband = get_absolute_time();   //  Baseline for LED green (packet received) blink duration
}

/*
 * Main SBand operation loop (RX/TX timing)
 * Handles packet reception and transmission at regular intervals
 */
void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX) {
    // SBAND_RX: Process packet queue (filled by ISR) - ONLY if not transmitting
    if (!Sband_Transmitting && radio_initialized == 0 &&
        absolute_time_diff_us(previous_time_Sband_RX, get_absolute_time()) >= interval_Sband_RX) {
        previous_time_Sband_RX = get_absolute_time();
        sprintf(buffer_Sband_RX, "SB_RXd ");  // using suffixes for version identification

        if (nCRC_sband > 0) {
            sprintf(buffer_Sband_RX + strlen(buffer_Sband_RX), "%d CRC ", nCRC_sband);
            nCRC_sband = 0;  // Zero out CRC error count after displaying  PHM this is probably not working right
        }

        // Process all packets in queue
        while (queue_count_sband > 0) {
            volatile packet_sband_t *pkt = &packet_queue_sband[queue_tail_sband];

            // Parse power value from packet
            // Format: "\xFF\xFF\xFF\xFFTX Power = %02d"
            int power = 0;
            if (sscanf((char*)pkt->data + 4, "TX Power = %d", &power) == 1) {
                // Adjust for SX1280 power range (-18 to +13 dBm)
                int hist_index = power + 18;  // Shift so -18 maps to 0
                if (hist_index >= 0 && hist_index < 32) {
                    power_histogram_sband[hist_index]++;
                } else {
                    printf("SBand Warning: Received out-of-range power value: %d\n", power);
                }
            } else {
                printf("SBand Warning: Failed to parse power from packet\n");
            }

            // Update queue
            queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;
            queue_count_sband--;
        }

        // Print the power histogram into buffer_Sband_RX
        int remaining_space = BUFLEN*2 - strlen(buffer_Sband_RX) - 1;
        for (int i = 0; (i < 32) && (remaining_space > 0); i++) {
            int written = snprintf(buffer_Sband_RX + strlen(buffer_Sband_RX), remaining_space,
                                  "%d ", power_histogram_sband[i]);
            if (written < 0 || written >= remaining_space) {
                printf("SBand Warning: Buffer overflow while writing histogram\n");
                break;
            }
            remaining_space -= written;
        }
    }

    // SBAND_TX: State machine for non-blocking transmission
    // Check if it's time to start a new transmission cycle
    if (!Sband_Transmitting && radio_initialized == 0 &&
        absolute_time_diff_us(previous_time_Sband_TX, get_absolute_time()) >= interval_Sband_TX) {
        // Start transmission cycle
        previous_time_Sband_TX = get_absolute_time();
        Sband_Transmitting = true;
        current_tx_power_sband = 10;  // Start at 10 dBm

        // Disable ISR during TX
        gpio_set_irq_enabled(SAMWISE_SBAND_D0_PIN, GPIO_IRQ_EDGE_RISE, false);
    }

    // If currently transmitting, send one packet per invocation
    if (Sband_Transmitting) {
        sband_set_tx_params(current_tx_power_sband, 0x02);  // Set power, ramp 20μs

        // Create packet: preamble + power + spaces to fill 250 bytes
        memset(tx_packet_sband, ' ', 250);  // Fill with spaces
        tx_packet_sband[0] = '\xFF';
        tx_packet_sband[1] = '\xFF';
        tx_packet_sband[2] = '\xFF';
        tx_packet_sband[3] = '\xFF';
        // Format power with leading zero or minus sign
        sprintf((char*)tx_packet_sband + 4, "TX Power = %02d", current_tx_power_sband);
        // Restore spaces after sprintf's null terminator
        for (int j = strlen((char*)tx_packet_sband); j < 250; j++) {
            tx_packet_sband[j] = ' ';
        }

        sband_packet_to_fifo(tx_packet_sband, 250);
        sband_transmit();  // Send the packet

        sprintf(buffer_Sband_TX, "Now sending TX Power = %02d\n", current_tx_power_sband);

        // Wait for TX completion. PHM Could also check for timeout on next invocation and before transmitting again.
        int timeout = 100000;
        while (!sband_tx_done() && timeout--) { sleep_us(10); }
        if (!sband_tx_done()) printf("SBand: TX timed out at power %d\n", current_tx_power_sband);

        // Move to next power level
        current_tx_power_sband--;

        // Check if transmission cycle is complete
        if (current_tx_power_sband < -1) {              // PHM I wonder if this should be a symbolic, or even command-able?
            // Transmission cycle complete
            Sband_Transmitting = false;
            current_tx_power_sband = 10;  // Reset for next cycle

            // Re-enable ISR and return to RX mode
            gpio_set_irq_enabled(SAMWISE_SBAND_D0_PIN, GPIO_IRQ_EDGE_RISE, true);
            sband_listen();

            buffer_Sband_TX[0] = '\0';  //PHM Zero the buffer out
        }
    }

    // Update LED color contribution based on SBand state
    if (Sband_Transmitting) {
        // Transmitting: Blue
        sband_led_r = 0;
        sband_led_g = 0;
        sband_led_b = 0x10;
    } else if (sband_rx_active && absolute_time_diff_us(sband_last_rx_time, get_absolute_time()) < 2000000) {
        // Receiving (within 2 seconds of last packet): Magenta (Red + Blue)
        sband_led_r = 0x10;
        sband_led_g = 0;
        sband_led_b = 0x10;
    } else {
        // Listening: Cyan (Green + Blue)
        sband_led_r = 0;
        sband_led_g = 0x08;
        sband_led_b = 0x10;
    }
}
