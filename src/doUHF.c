#include "doUHF.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "ws2812.h"  // For LED control (red, white)
#include "main_gps_uart_shared_buffer.h"  // For BUFLEN definition

// RFM96 function declarations are in pins.h (already included via doUHF.h)
// RFM96 register definitions (from rfm96.c)
#define _RH_RF95_REG_12_IRQ_FLAGS 0x12

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

// Timing variables
static absolute_time_t previous_time_RADIO_RX;
static absolute_time_t previous_time_RADIO_TX;
static uint32_t interval_RADIO_RX = 1000000;        // 1 second
static uint32_t interval_RADIO_TX = 20*1010*1000;   // 20.2 seconds

// State machine variables for non-blocking TX
static bool UHF_Transmitting = false;
static int current_tx_power_uhf = 10;  // Start at 10 dBm, count down to -1
static bool uhf_rx_active = false;     // Track if we received packets
static absolute_time_t uhf_last_rx_time;

// Static buffer for TX packets
static uint8_t tx_packet[250];

// LED color contribution from UHF (for additive color mixing)
uint8_t uhf_led_r = 0;
uint8_t uhf_led_g = 0;
uint8_t uhf_led_b = 0;

/*
 * DIO0 GPIO Interrupt Service Routine
 * Triggered when a packet is received (RX_DONE interrupt)
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

            // Mark RX as active for LED color management
            uhf_rx_active = true;
            uhf_last_rx_time = get_absolute_time();
        }

        // Clear RX_DONE interrupt flag
        rfm96_put8(_RH_RF95_REG_12_IRQ_FLAGS, 0x40);

        // Continue listening for next packet
        rfm96_listen();
    }
    // Note: If TX_DONE fires (bit 3), we ignore it - ISR is only for RX
}

/*
 * Initialize UHF radio with tx_done test
 * Includes ISR setup and tx_done timing test
 */
void initUHF(spi_pins_t *spi_pins) {
    // Initialize radio
    int radio_initialized = rfm96_init(spi_pins);

    if (radio_initialized) {
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

        // Enable ISR after test completes
        gpio_set_irq_enabled_with_callback(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true, &dio0_isr);
        printf("UHF: DIO0 ISR enabled on GPIO %d\n", SAMWISE_RF_D0_PIN);
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
    if (!UHF_Transmitting &&
        absolute_time_diff_us(previous_time_RADIO_RX, get_absolute_time()) >= interval_RADIO_RX) {
        previous_time_RADIO_RX = get_absolute_time();
        sprintf(buffer_RADIO_RX, "RXdd ");  // DMA, Non-Blocking Clears out the results buffer

        if (nCRC > 0) {
            sprintf(buffer_RADIO_RX + strlen(buffer_RADIO_RX), "%d CRC ", nCRC);
            nCRC = 0;  // Zero out CRC error count after displaying
        }

        // Process all packets in queue
        while (queue_count > 0) {
            volatile packet_t *pkt = &packet_queue[queue_tail];

            // Parse power value from packet
            // Format: "\xFF\xFF\xFF\xFFTX Power = %02d"
            int power = 0;
            if (sscanf((char*)pkt->data + 4, "TX Power = %d", &power) == 1) {
                // Adjust for negative power values (range: -1 to 17)
                int hist_index = power + 1;  // Shift so -1 maps to 0
                if (hist_index >= 0 && hist_index < 20) {
                    power_histogram[hist_index]++;
// PHM:  We know when the we will receive the last packet, if we hear it
//      So let's set the transmitter to start after that.
//      previous_time_RADIO_TX = get_absolute_time() - interval_RADIO_TX + TX_DURATION * hist_index + 100000; // 0.1s after last RX
                } else {
                    printf("UHF Warning: Received out-of-range power value: %d\n", power);
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
    }

    // RADIO_TX: State machine for non-blocking transmission
    // Check if it's time to start a new transmission cycle
    if (!UHF_Transmitting &&
        absolute_time_diff_us(previous_time_RADIO_TX, get_absolute_time()) >= interval_RADIO_TX) {
        // Start transmission cycle
        previous_time_RADIO_TX = get_absolute_time();
        UHF_Transmitting = true;
        current_tx_power_uhf = 10;  // Start at 10 dBm

        // Disable ISR during TX
        gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, false);
    }

    // If currently transmitting, send one packet per invocation
    if (UHF_Transmitting) {
        rfm96_set_tx_power(current_tx_power_uhf);

        // Create packet: preamble + power + spaces to fill 250 bytes
        memset(tx_packet, ' ', 250);  // Fill with spaces
        tx_packet[0] = '\xFF';
        tx_packet[1] = '\xFF';
        tx_packet[2] = '\xFF';
        tx_packet[3] = '\xFF';
        // Format power with leading zero or minus sign
        sprintf((char*)tx_packet + 4, "TX Power = %02d", current_tx_power_uhf);
        // Restore spaces after sprintf's null terminator
        for (int j = strlen((char*)tx_packet); j < 250; j++) {
            tx_packet[j] = ' ';
        }

        rfm96_packet_to_fifo(tx_packet, 250);
        rfm96_transmit();  // Send the packet

        // Wait for TX completion
        int timeout = 100000;
        while (!rfm96_tx_done() && timeout--) { sleep_us(10); }
        if (!rfm96_tx_done()) printf("UHF: TX timed out at power %d\n", current_tx_power_uhf);

        // Move to next power level
        current_tx_power_uhf--;

        // Check if transmission cycle is complete
        if (current_tx_power_uhf < -1) {
            // Transmission cycle complete
            UHF_Transmitting = false;
            current_tx_power_uhf = 10;  // Reset for next cycle

            // Re-enable ISR and return to RX mode
            gpio_set_irq_enabled(SAMWISE_RF_D0_PIN, GPIO_IRQ_EDGE_RISE, true);
            rfm96_listen();

            sprintf(buffer_RADIO_TX, "RADIO_TXdd packets sent (10 to -1 dBm)\n");
        }
    }

    // Update LED color contribution based on UHF state
    if (UHF_Transmitting) {
        // Transmitting: Red
        uhf_led_r = 0x10;
        uhf_led_g = 0;
        uhf_led_b = 0;
    } else if (uhf_rx_active && absolute_time_diff_us(uhf_last_rx_time, get_absolute_time()) < 2000000) {
        // Receiving (within 2 seconds of last packet): Green
        uhf_led_r = 0;
        uhf_led_g = 0x08;
        uhf_led_b = 0;
    } else {
        // Listening: Yellow (Red + Green)
        uhf_led_r = 0x10;
        uhf_led_g = 0x08;
        uhf_led_b = 0;
    }
}
