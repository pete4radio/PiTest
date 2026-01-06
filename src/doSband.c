#include "doSband.h"
#include "doUHF.h"    // For UHF_TX global state
#include "sband.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "ws2812.h"  // For LED control (red, white)
#include "main_gps_uart_shared_buffer.h"  // For BUFLEN definition

/*  

Called by main.c and calling sband.c, this function implements SBand superloop
functionality for PiTest.

RX is DIO1 ISR driven, using a packet queue to fill a histogram of transmitted power levels.
Non-blocking TX is implemented sends one packet at new power level each invocation.
LoRa Parameters are hard coded.
Lively LED color indication of received packets and transmissions.
Both this radio and the one at the other end of the link are running identical code.

*/

// Serialize/deserialize uint32_t (little-endian -- pico-sdk, intel, etc's native)
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
volatile absolute_time_t last_packet_time_sband;  // For LED blue timing

// Global CRC error counter (similar to UHF)
volatile uint8_t nCRC_sband = 0;

// Power histogram for received power levels
static int power_histogram_sband[32] = {0};  // histogram for -18 to +13 dBm range

// Packet set counters
static uint32_t sband_packet_set_count = 0;        // TX counter
static uint32_t sband_last_rx_packet_set_count = 0; // RX counter
static uint32_t sband_last_printed_packet_set_count = 0; // Last count when stats were printed
static uint32_t sband_rx_packet_set_offset = 0;    // Offset to handle late boot synchronization
static bool sband_offset_initialized = false;       // Track if offset has been set

// Timing variables
static absolute_time_t previous_time_Sband_RX;
static absolute_time_t previous_time_Sband_TX;
static uint32_t interval_Sband_RX = 1000000;        // 1 second
static uint32_t interval_Sband_TX = 20*1010*1000;   // 20.2 seconds

// State machine variables for non-blocking TX
// Note: Sband_Transmitting is controlled by global UHF_TX state (!UHF_TX = Sband RX)
static int current_tx_power_sband = SBAND_MAX_POWER;  // Start at max power, count down to min
volatile absolute_time_t sband_last_rx_time = 0;    // Timestamp of last packet (for LED logic)
static bool radio_initialized = false;              // sband radio initialized flag (true = initialized, false = not initialized)

// Static buffer for TX packets
static uint8_t tx_packet_sband[250];

// RX mode selection: Set to 1 for interrupt-driven, 0 for polling
#define SBAND_RX_USE_INTERRUPTS 0  // Change to 1 to use DIO1 interrupts

/*
 * DIO1 GPIO Interrupt Service Routine for SBand
 * Triggered when a packet is received (RX_DONE interrupt)
 * Copies packet from radio FIFO to queue
 */
void sband_dio1_isr(uint gpio, uint32_t events) {
    // Check IRQ status
    uint16_t irq_flags = sband_get_irq_status();

    // Debug: Print IRQ flags when ISR is called
    static uint32_t isr_count = 0;
//    printf("SBand ISR #%lu: IRQ=0x%04X\n", ++isr_count, irq_flags);

    if (irq_flags & SX1280_IRQ_RX_DONE) {
//        printf("SBand: RX_DONE detected!\n");       //PHM does printf even work from an ISR?
        blue(); // no delay in telling the user the good news - a packet has arrived!
        last_packet_time_sband = get_absolute_time();  // Turn it off after seen
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

#define TRY_INIT 2  // Maximum initialization attempts

/*
 * Attempt to initialize SBand radio with retry logic
 * Includes TX validation tests
 * Returns: true if initialization successful, false if all retries exhausted
 */
static bool attempt_sband_init(void) {
    yellow();
    for (int attempt = 1; attempt <= TRY_INIT; attempt++) {
        if (attempt > 1) {
            printf("SBand: Retry attempt %d/%d...\n", attempt, TRY_INIT);
            sleep_ms(1000);  // Delay between retries
        }

        // Attempt basic initialization
        if (sband_init() != 0) {
            printf("SBand: Init attempt %d/%d failed in sband_init()\n",
                   attempt, TRY_INIT);
            continue;  // Try again
        }

        printf("SBand: Init attempt %d/%d: sband_init() succeeded, running validation tests...\n",
               attempt, TRY_INIT);

        // Disable ISR during TX validation tests
        gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, false);

        // TX Validation Test 1: Short packet
        cyan();  // Indicate transmitting
        absolute_time_t start_time = get_absolute_time();

        uint8_t short_data[5] = {0xFF, 0xAA, 0x55, 0xBB, 0xCC};
        sband_packet_to_fifo(short_data, 5);
        sband_transmit();

        int ip = 10000;
        while (!sband_tx_done() && ip--) { sleep_us(10); }

        if (!sband_tx_done()) {
            printf("SBand: Init attempt %d/%d failed: TX validation timeout (short packet)\n",
                   attempt, TRY_INIT);
            continue;  // Try again
        }

        printf("SBand: Short packet TX: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        // TX Validation Test 2: Long packet
        start_time = get_absolute_time();

        uint8_t long_data[250];
        memset(long_data, 0xAA, 250);
        sband_packet_to_fifo(long_data, 250);
        sband_transmit();

        ip = 100000;
        while (!sband_tx_done() && ip--) { sleep_us(10); }

        if (!sband_tx_done()) {
            printf("SBand: Init attempt %d/%d failed: TX validation timeout (long packet)\n",
                   attempt, TRY_INIT);
            continue;  // Try again
        }

        printf("SBand: Long packet TX: %lld ms (%d iterations left)\n",
               absolute_time_diff_us(start_time, get_absolute_time()) / 1000, ip);

        // All tests passed!
        printf("SBand: Init attempt %d/%d: ALL VALIDATION TESTS PASSED\n",
               attempt, TRY_INIT);

        sband_listen();  // Set to RX mode
        magenta();       // Indicate receiving

        return true;  // Success!
    }

    // All retries exhausted
    printf("SBand: ERROR: Initialization failed after %d attempts\n", TRY_INIT);
    return false;
}

/*
 * Initialize SBand radio with tx_done test
 * Includes ISR setup and tx_done timing test
 */
void initSband(void) {
    // Attempt initialization with retries
    radio_initialized = attempt_sband_init();

    if (radio_initialized) {
        printf("SBand: Radio initialization SUCCESSFUL\n");
        printf("SBand: DIO1 ISR will be enabled via unified dispatcher in main.c (GPIO %d)\n",
               SAMWISE_SBAND_D1_PIN);
    } else {
        printf("SBand: Radio initialization FAILED - radio will remain disabled\n");
    }

    // Initialize timing variables regardless of init success
    // (doSband() checks radio_initialized before using radio)
    previous_time_Sband_RX = get_absolute_time();   //  Baseline for checking the RX queue
    previous_time_Sband_TX = get_absolute_time();   //  Baseline for starting TX cycles
    last_packet_time_sband = get_absolute_time();   //  Baseline for LED green (packet received) blink duration
}

/*
 * Main SBand operation loop (RX/TX timing)
 * Handles packet reception and transmission at regular intervals
 */
void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX) {
    // Handle UHF_TX state changes FIRST (must happen even if radio not initialized)
    // This allows re-initialization attempts when transitioning to TX mode
    static bool prev_UHF_TX = true;  // Start assuming UHF is TX (Sband is RX)
    static bool first_call = true;

    // Debug: Show state on first call
    if (first_call) {
        printf("SBand: doSband() first call - UHF_TX=%d, radio_initialized=%d\n",
               UHF_TX, radio_initialized);
        first_call = false;
    }

    if (UHF_TX != prev_UHF_TX) {
        printf("SBand: UHF_TX changed %d->%d, radio_init=%d\n",
               prev_UHF_TX, UHF_TX, radio_initialized);
        if (UHF_TX) {
            // UHF_TX went from false to true: Stop Sband TX, start Sband RX
            printf("SBand: Transition to RX mode (UHF is TX)\n");
            if (radio_initialized) {
                sband_listen();
                gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, true);
            } else {
                printf("SBand: Cannot enter RX - radio not initialized!\n");
            }
            buffer_Sband_TX[0] = '\0';  // Clear TX buffer
        } else {
            // UHF_TX went from true to false: Stop Sband RX, start Sband TX

            // If radio not initialized, attempt re-initialization
            if (!radio_initialized) {
                printf("SBand: Radio not initialized, attempting re-initialization...\n");
                radio_initialized = attempt_sband_init();

                if (radio_initialized) {
                    printf("SBand: Re-initialization SUCCESSFUL\n");
                } else {
                    printf("SBand: Re-initialization FAILED - skipping S-band operations\n");
                }
            }

            // Only proceed with TX setup if radio is initialized
            if (radio_initialized) {
                gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, false);
                sband_set_mode(SX1280_MODE_STDBY_RC);
                current_tx_power_sband = SBAND_MAX_POWER;  // Start at max power
            }
        }
        prev_UHF_TX = UHF_TX;
    }

#if !SBAND_RX_USE_INTERRUPTS
    // Polling mode: Check for RX_DONE every loop iteration
    if (radio_initialized && UHF_TX) {
        if (sband_rx_done()) {
            printf("SBand: RX_DONE detected via polling\n");
            blue();  // Show packet arrival
            last_packet_time_sband = get_absolute_time();

            // Check if queue has space
            if (queue_count_sband < QUEUE_SIZE_SBAND) {
                volatile packet_sband_t *pkt = &packet_queue_sband[queue_head_sband];

                // Read packet from FIFO
                pkt->length = sband_packet_from_fifo((uint8_t*)pkt->data);

                // Get SNR and RSSI
                pkt->snr = sband_get_snr();
                pkt->rssi = sband_get_rssi();

                // Update SNR and RSSI in packet
                if (pkt->length > 25) {
                    snprintf((char*)&pkt->data[25], 25, "SNR = %4d; RSSI = %4d",
                             (int)pkt->snr, (int)pkt->rssi);
                }

                // Update queue
                queue_head_sband = (queue_head_sband + 1) % QUEUE_SIZE_SBAND;
                queue_count_sband++;

                printf("SBand: Packet received, len=%d, SNR=%d, RSSI=%d\n",
                       pkt->length, pkt->snr, pkt->rssi);
            }

            // Clear RX_DONE and continue listening
            sband_clear_irq_status(SX1280_IRQ_RX_DONE);
            sband_listen();
        }
    }
#endif

    // Skip all radio operations if not initialized
    if (!radio_initialized) {return;}

    // SBAND_RX: Process packet queue (filled by ISR) - ONLY when UHF is transmitting
    if (UHF_TX && radio_initialized &&
        absolute_time_diff_us(previous_time_Sband_RX, get_absolute_time()) >= interval_Sband_RX) {
        previous_time_Sband_RX = get_absolute_time();
        sprintf(buffer_Sband_RX, "SB_RX ");  // using suffixes for version identification

        if (nCRC_sband > 0) {
            sprintf(buffer_Sband_RX + strlen(buffer_Sband_RX), "%d CRC ", nCRC_sband);
            nCRC_sband = 0;  // Zero out CRC error count after displaying  PHM this is probably not working right
        }

        // Process all packets in queue
        while (queue_count_sband > 0) {
            volatile packet_sband_t *pkt = &packet_queue_sband[queue_tail_sband];
            blue();
            // Parse power value from packet
            // Format: "TX Power = %02d" (preamble stripped by SX1280)
            int power = 0;
            if (sscanf((char*)pkt->data, "TX Power = %d", &power) == 1) {
                // Extract packet set counter from bytes 16-19 (20-4 for UNstripped preamble)
                // The preamble is stripped off by the SX1280 hardware, but the address bytes
                // remain, so the counter starts at byte 20 in the received data
                if (pkt->length >= 24) {
                    uint32_t rx_packet_set_count = read_uint32_le(pkt->data + 20);
                    sband_last_rx_packet_set_count = rx_packet_set_count;

                    // Histogram begins recording after both RX and TX have started
                    //  to avoid mis-alignment due to late starts
                    if (rx_packet_set_count == 0) {
                        // The distant Transmitter rebooted - reset our histogram and offset
                        for (int i = 0; i < 32; i++) {
                            power_histogram_sband[i] = 0;
                        }
                        sband_rx_packet_set_offset = 0;
                        sband_offset_initialized = false;
                    } else if (!sband_offset_initialized) {
                        // First packet received with count > 0 - save offset
                        sband_rx_packet_set_offset = rx_packet_set_count;
                        sband_offset_initialized = true;
                    }
                }

                // SBand receives from another identical SBand radio (same band)
                // Flash the same code to both radios
                int hist_index = power - SBAND_MIN_POWER;  // Map SBand min power to index 0
                if (hist_index >= 0 && hist_index < 32) {
                    power_histogram_sband[hist_index]++;
                } else {
                    printf("SBand Warning: Received out-of-range power value: %d (expected SBand range %d-%d)\n",
                           power, SBAND_MIN_POWER, SBAND_MAX_POWER);
                }
            } else {
                printf("SBand Warning: Failed to parse power from packet\n");
                printf("Packet data (len=%d): ", pkt->length);
                for (int i = 0; i < pkt->length; i++) {
                    printf("%02X ", pkt->data[i]);
                }
                printf("\n");
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

        // Calculate and display packet loss statistics (only when count changes)
        if (sband_last_rx_packet_set_count > 0 &&
            sband_last_rx_packet_set_count != sband_last_printed_packet_set_count) {
            printf("\n=== SBand Packet Loss Statistics (Set #%lu) ===\n", sband_last_rx_packet_set_count);
            printf("Power(dBm) | Expected | Received | Lost | Loss%% | log10(loss)\n");
            printf("-----------|----------|----------|------|--------|------------\n");

            for (int i = 0; i < 32; i++) {
                int power_dbm = SBAND_MIN_POWER + i;  // SBand receives from another SBand

                // Only show power levels in SBand's TX sweep range
                if (power_dbm < SBAND_MIN_POWER || power_dbm > SBAND_MAX_POWER) continue;

                // Calculate expected based on offset-corrected count
                uint32_t expected = sband_last_rx_packet_set_count - sband_rx_packet_set_offset;
                uint32_t received = power_histogram_sband[i];
                uint32_t lost = (received > expected) ? 0 : (expected - received);
                float loss_fraction = (float)lost / (float)expected;
                float loss_percent = loss_fraction * 100.0f;

                // Calculate log10(loss_fraction) with floor at -2.0
                float log10_loss;
                if (loss_fraction < 0.01f) {
                    log10_loss = -2.0f;  // Floor for excellent reception
                } else if (loss_fraction < 0.1f) {
                    log10_loss = -1.0f;  // 1-10% loss
                } else if (loss_fraction < 1.0f) {
                    log10_loss = -0.3f;  // 10-100% loss (approximation)
                } else {
                    log10_loss = 0.0f;   // 100% loss
                }

                printf("%10d | %8lu | %8d | %4lu | %5.1f%% | %10.2f\n",
                       power_dbm, expected, received, lost, loss_percent, log10_loss);
            }
            printf("====================================================\n\n");

            // Update last printed count
            sband_last_printed_packet_set_count = sband_last_rx_packet_set_count;
        }
    }

    // SBAND_TX: Transmit when UHF is in RX mode (!UHF_TX)
    // If currently transmitting (UHF in RX mode), send one packet per invocation
    if (!UHF_TX && radio_initialized) {
        sband_set_tx_params(current_tx_power_sband, 0x02);  // Set power, ramp 20μs

        // Create packet: preamble + power + spaces to fill 250 bytes
        memset(tx_packet_sband, ' ', 250);  // Fill with spaces
        tx_packet_sband[0] = '\xFF';
        tx_packet_sband[1] = '\xFF';
        tx_packet_sband[2] = '\xFF';
        tx_packet_sband[3] = '\xFF';
        // Format power with leading zero or minus sign
        sprintf((char*)tx_packet_sband + 4, "TX Power = %02d", current_tx_power_sband);
        // Add packet set counter at fixed position (bytes 20-23, after power string)
        write_uint32_le(tx_packet_sband + 20, sband_packet_set_count);

        // Copy current SBand RX display buffer into packet after the counter
        size_t s_offset = 24;  // After preamble (4) + power string (14) + counter (4) + padding (2)
        size_t s_copy_len = strlen(buffer_Sband_RX);
        if (s_copy_len > (size_t)(250 - s_offset)) s_copy_len = 250 - s_offset;
        if (s_copy_len > 0) {
            memcpy(tx_packet_sband + s_offset, buffer_Sband_RX, s_copy_len);
        }

        // Wait for TX completion. PHM Could also check for timeout on next invocation and before transmitting again.
        // Clear any stale IRQ flags, write packet and transmit
        sband_clear_irq_status(0xFFFF);
        sband_packet_to_fifo(tx_packet_sband, 250);
        sband_transmit();  // Send the packet; inform the LED
        white();  // Indicate transmitting, forces UHF to re-indicate received packets
        sprintf(buffer_Sband_TX, "Now sending TX Power = %02d\n", current_tx_power_sband);

        // Wait for TX completion by polling IRQ status directly (clear when seen)
        int timeout = 100000;
        uint16_t irq;
        while (timeout-- > 0) {
            irq = sband_get_irq_status();
            if (irq & SX1280_IRQ_TX_DONE) {
                sband_clear_irq_status(SX1280_IRQ_TX_DONE);
                break;
            }
            sleep_us(10);
        }
        if (timeout <= 0) {
            irq = sband_get_irq_status();
            printf("SBand: TX timed out waiting for TX_DONE at power %d (irq=0x%04X)\n", current_tx_power_sband, irq);
        }

        // Move to next power level
        current_tx_power_sband--;

        // Check if transmission cycle is complete - loop continuously until UHF_TX goes true
        if (current_tx_power_sband < SBAND_MIN_POWER) {
            sband_packet_set_count++;  // Increment on completing full sweep
            current_tx_power_sband = SBAND_MAX_POWER;  // Reset to start of cycle and continue
            // Note: Don't switch to RX mode here - that happens when UHF_TX changes
        }
    }
    // If our "packet received" LED color is stale, switch back to indicating UHF TX only.
    if (UHF_TX && absolute_time_diff_us(sband_last_rx_time, get_absolute_time()) > 2000000) {
        red();
    }
}
