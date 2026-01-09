#include "doUHF.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "ws2812.h"  // For LED control (red, white)
#include "main_gps_uart_shared_buffer.h"  // For BUFLEN definition

// RFM96 function declarations are in pins.h (already included via doUHF.h)
// RFM96 register definitions (from rfm96.c)
#define _RH_RF95_REG_12_IRQ_FLAGS 0x12

// Serialize/deserialize uint32_t (little-endian)
static inline void write_uint32_le(uint8_t *buf, uint32_t value) {
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
}

static inline uint32_t read_uint32_le(const volatile uint8_t *buf) {
    return ((uint32_t)buf[0]) | (((uint32_t)buf[1]) << 8) |
           (((uint32_t)buf[2]) << 16) | (((uint32_t)buf[3]) << 24);
}

// Packet queue structure definitions (for ISR access)
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

// Global CRC error counter (incremented by rfm96_packet_from_fifo)
// This is defined here and referenced in rfm96.c as extern
volatile uint8_t nCRC = 0;

// Power histogram for received power levels
static int power_histogram[20] = {0};  // histogram of received power levels

// Packet set counters
static uint32_t uhf_packet_set_count = 0;        // TX: incremented when completing full sweep
static uint32_t uhf_last_rx_packet_set_count = 0; // RX: extracted from received packets
static uint32_t uhf_last_printed_packet_set_count = 0; // Last count when stats were printed
static uint32_t uhf_rx_packet_set_offset = 0;    // Offset to handle late boot synchronization
static bool uhf_offset_initialized = false;       // Track if offset has been set

// Timing variables
static absolute_time_t previous_time_RADIO_RX;
static absolute_time_t previous_time_RADIO_TX;
static uint32_t interval_RADIO_RX = 1000000;        // 1 second
static uint32_t interval_RADIO_TX = 20*1010*1000;   // 20.2 seconds

// Global state machine: true = UHF TX (SBand RX), false = UHF RX (SBand TX)
volatile bool UHF_TX = false;

// State machine variables for non-blocking TX
static int current_tx_power_uhf = UHF_MAX_POWER;  // Start at max power, count down to min
static bool uhf_rx_active = false;     // Track if we received packets (deprecated - use uhf_last_rx_time)
volatile absolute_time_t uhf_last_rx_time = 0;  // Timestamp of last packet (for LED logic)

// Static buffer for TX packets
static uint8_t tx_packet[250];

// LED color contribution from UHF (for additive color mixing)
//volatile uint8_t uhf_led_r = 0;
//volatile uint8_t uhf_led_g = 0;
//volatile uint8_t uhf_led_b = 0;

/*
 * DIO0 GPIO Interrupt Service Routine
 * Triggered when a packet is received (RX_DONE interrupt base on SAMWISE_RF_D0_PIN)
 * Copies packet from radio FIFO to queue using non-blocking DMA
 */
void dio0_isr(uint gpio, uint32_t events) {
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
            green();

            // When a packet comes in, signal it on the LED for a short time
            uhf_last_rx_time = get_absolute_time();
        }

        // Clear RX_DONE interrupt flag (rfm96_listen will clear all flags)
        rfm96_put8(_RH_RF95_REG_12_IRQ_FLAGS, 0x40);

        // Continue listening for next packet (clears all IRQ flags, sets RX mode)
        rfm96_listen();
    }
    // Note: If TX_DONE fires (bit 3), we ignore it - ISR is only for RX, and during
    // TX we disable the ISR to prevent unnecessary interrupts.
}

/*
 * Initialize UHF radio with tx_done test
 * Includes ISR setup and tx_done timing test
 */
void initUHF(void) {
    // Initialize radio
    int radio_initialized = rfm96_init();

    if (radio_initialized == 0) {
        printf("UHF: Radio initialized\n");

        // Disable ISR during test to prevent interference with tx_done()
        gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, false);

        // Check that we can know here when the transmitter has completed
        // by sending a short and a long packet and measuring the time it takes

        // Test 1: Short packet
        red();  // Indicate we are transmitting
        absolute_time_t start_time = get_absolute_time();

        // Send a short packet (5 bytes)
        char short_data[5] = "TEST";
        rfm96_packet_to_fifo((uint8_t*)short_data, 5);
        rfm96_transmit();  // Send the packet

        int ip = 10000;
        while (!rfm96_tx_done() && ip--) { sleep_us(10); }
        if (!rfm96_tx_done()) printf("UHF: TX timed out (short)\n");
        printf("UHF: Time to send a short packet: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        // Test 2: Long packet
        start_time = get_absolute_time();

        // Send a long packet (250 bytes)
        char long_data[250];
        memset(long_data, 'A', 250);
        rfm96_packet_to_fifo((uint8_t*)long_data, 250);
        rfm96_transmit();  // Send the packet

        ip = 100000;
        while (!rfm96_tx_done() && ip--) { sleep_us(10); }
        if (!rfm96_tx_done()) printf("UHF: TX timed out (long)\n");
        printf("UHF: Time to send a long packet: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        rfm96_listen(); // Set the radio to RX mode
        white();    // Indicate we are receiving

        // Note: IRQ will be enabled in main.c via unified dispatcher (RP2040 limitation)
        printf("UHF: DIO0 ISR will be enabled via unified dispatcher in main.c (GPIO %d)\n", SAMWISE_RF_D0_PIN);
    } else {
        printf("UHF: Radio initialization failed\n");
    }

    // Initialize timing variables
    previous_time_RADIO_RX = get_absolute_time();
    previous_time_RADIO_TX = get_absolute_time();
    last_packet_time = get_absolute_time();
}

/*
 * Main UHF operation loop (RX/TX timing)
 * Handles packet reception and transmission at regular intervals
 */
void doUHF(char *buffer_RADIO_RX, char *buffer_RADIO_TX) {
    // RADIO_RX: Process packet queue (filled by ISR) - ONLY if not transmitting
    if (!UHF_TX &&
        absolute_time_diff_us(previous_time_RADIO_RX, get_absolute_time()) >= interval_RADIO_RX) {
        previous_time_RADIO_RX = get_absolute_time();
        sprintf(buffer_RADIO_RX, "URX ");  // Clears out the prior results buffer

        if (nCRC > 0) {
            sprintf(buffer_RADIO_RX + strlen(buffer_RADIO_RX), "%d CRC ", nCRC);
            nCRC = 0;  // Zero out CRC error count after displaying
        }

        // Process all packets in queue
        while (queue_count > 0) {
            volatile packet_t *pkt = &packet_queue[queue_tail];

//  A packet has been received
            green();

            // Parse power value from packet.  Format: "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFFTX Power = %02d"
            int power = 0;
            if (sscanf((char*)pkt->data + 8, "TX Power = %d", &power) == 1) {
                // Extract packet set counter from bytes 24-27 (after network bytes and power string)
                if (pkt->length >= 28) {
                    uint32_t rx_packet_set_count = read_uint32_le(pkt->data + 24);
                    uhf_last_rx_packet_set_count = rx_packet_set_count;

                    // Handle synchronization
                    if (rx_packet_set_count == 0) {
                        // Transmitter rebooted - reset histogram and offset
                        for (int i = 0; i < 20; i++) {
                            power_histogram[i] = 0;
                        }
                        uhf_rx_packet_set_offset = 0;
                        uhf_offset_initialized = false;
                    } else if (!uhf_offset_initialized) {
                        // First packet received with count > 0 - save offset
                        uhf_rx_packet_set_offset = rx_packet_set_count;
                        uhf_offset_initialized = true;
                    }
                }

                // UHF receives from another UHF radio (same band)
                int hist_index = power - UHF_MIN_POWER;  // Map UHF min power to index 0
                if (hist_index >= 0 && hist_index < 20) {
                    power_histogram[hist_index]++;
// We know when the we will receive the last packet, if we hear it
//      So let's set the transmitter to start after that.
                    previous_time_RADIO_TX = get_absolute_time() - interval_RADIO_TX +\
                     (UHF_TIME_ON_THE_AIR * 1000) * hist_index + 100000; // 0.1s after last RX
                } else {
                    printf("UHF Warning: Received out-of-range power value: %d (expected UHF range %d-%d)\n",
                           power, UHF_MIN_POWER, UHF_MAX_POWER);
                }
            } else {
                printf("UHF Warning: Failed to parse power from packet\n");
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
                printf("UHF Warning: Buffer overflow while writing histogram\n");
                break;
            }
            remaining_space -= written;
        }

        // Calculate and display packet loss statistics (only when count changes)
        if (uhf_last_rx_packet_set_count > 0 &&
            uhf_last_rx_packet_set_count != uhf_last_printed_packet_set_count) {
            printf("\n=== UHF Packet Loss Statistics (Set #%lu) ===\n", uhf_last_rx_packet_set_count);
            printf("Power(dBm) | Expected | Received | Lost | Loss%%\n");
            printf("-----------|----------|----------|------|--------\n");

            for (int i = 0; i < 20; i++) {
                int power_dbm = UHF_MIN_POWER + i;  // UHF receives from another UHF

                // Only show power levels in UHF's TX sweep range
                if (power_dbm < UHF_MIN_POWER || power_dbm > UHF_MAX_POWER) continue;

                // Calculate expected based on offset-corrected count
                uint32_t expected = uhf_last_rx_packet_set_count - uhf_rx_packet_set_offset;
                uint32_t received = power_histogram[i];
                uint32_t lost = (received > expected) ? 0 : (expected - received);
                float loss_fraction = (float)lost / (float)expected;
                float loss_percent = loss_fraction * 100.0f;

                printf("%10d | %8lu | %8d | %4lu | %5.1f%%\n",
                       power_dbm, expected, received, lost, loss_percent);
            }
            printf("====================================================\n\n");

            // Update last printed count
            uhf_last_printed_packet_set_count = uhf_last_rx_packet_set_count;
        }
    }

    // RADIO_TX: State machine for non-blocking transmission

    // If currently transmitting, send one packet per invocation
    if (UHF_TX) {
        red();  // Indicate transmitting to user
        rfm96_set_tx_power(current_tx_power_uhf);

        // Create packet: preamble + 4 network bytes + power + spaces to fill 250 bytes
        memset(tx_packet, ' ', 250);  // Fill with spaces
        tx_packet[0] = '\xFF';
        tx_packet[1] = '\xFF';
        tx_packet[2] = '\xFF';
        tx_packet[3] = '\xFF';
        // Network bytes: dest, src, ack, serial - initialize to broadcast/unknown (0xFF)
        tx_packet[4] = 0xFF; // dest
        tx_packet[5] = 0xFF; // src
        tx_packet[6] = 0xFF; // ack
        tx_packet[7] = 0xFF; // serial
        // Format power with leading zero or minus sign (moved after network bytes)
        sprintf((char*)tx_packet + 8, "TX Power = %02d", current_tx_power_uhf);
        // Add packet set counter at fixed position (bytes 24-27, shifted by 4 bytes)
        write_uint32_le(tx_packet + 24, uhf_packet_set_count);

        // Copy current RX display buffer into packet after the counter
        size_t offset = 28;  // After preamble (4) + network (4) + power string (14) + counter (4)
        size_t copy_len = strlen(buffer_RADIO_RX);
        if (copy_len > (size_t)(250 - offset)) copy_len = 250 - offset;
        if (copy_len > 0) {
            memcpy(tx_packet + offset, buffer_RADIO_RX, copy_len);
        }

        rfm96_packet_to_fifo(tx_packet, 250);
        rfm96_transmit();  // Send the packet
        red();  // Indicate transmitting

        sprintf(buffer_RADIO_TX, "doUHF: Now sending TX Power = %02d\n", current_tx_power_uhf);

        // Wait for TX completion
        int timeout = 100000;
        while (!rfm96_tx_done() && timeout--) { sleep_us(10); }
        if (!rfm96_tx_done()) printf("UHF: TX timed out at power %d\n", current_tx_power_uhf);

        // Move to next power level
        current_tx_power_uhf--;

        // Check if transmission cycle is complete
        if (current_tx_power_uhf < UHF_MIN_POWER) {
            // Increment packet set counter (completed full sweep)
            uhf_packet_set_count++;

            // Transmission cycle complete - switch to SBand TX, UHF RX
            UHF_TX = false;
            current_tx_power_uhf = UHF_MAX_POWER;  // Reset for next cycle
        //  If the power supply can provide 1.5A, set this and the Sband
        //  parameter of the same name to 20.  PHM

            // Return to RX mode BEFORE re-enabling ISR (clears DIO0 from TX_DONE)
            rfm96_listen();
            white();  // Indicate listening
            gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true);

            buffer_RADIO_TX[0] = '\0';  // Zero the buffer out
            previous_time_RADIO_TX = get_absolute_time();   // Reset TX timer
        }
    } // end if (UHF_TX)

    // Not UHF transmitting: Check if it's time to start a new transmission cycle
    if (!UHF_TX &&
        absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
        // Start transmission cycle - switch to UHF TX, SBand RX
        previous_time_RADIO_TX = get_absolute_time();
        UHF_TX = true;
        current_tx_power_uhf = UHF_MAX_POWER;  // Start at max power

        // Disable ISR during TX
        gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, false);
    }

    // If it's been a while since last RX packet, signal to user
     if (!UHF_TX && absolute_time_diff_us(uhf_last_rx_time, get_absolute_time()) > 500000) {
        white();
    }
}