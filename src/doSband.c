#include "doSband.h"
#include "doUHF.h"    // For UHF_TX global state
#include "sband.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
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

/*
Functions (summary):
- sband_dio1_isr: GPIO DIO1 ISR; reads IRQ status, copies FIFO into packet queue, stores length/SNR/RSSI into
    the received buffer, updates queue indices and clears IRQs. Minimal work is done here, no printf.
- attempt_sband_init: initialization with retry and TX validation tests (short/long packets), disables ISR
    during validation, and reports status via LEDs/printf.
- initSband: high-level radio initialization wrapper (calls sband_init, configures pins/IRQ).
- doSband: main SBand service loop called from core1; processes queued packets, updates histogram,
    performs non-blocking TX scheduling and state machine for power stepping.
- sband_print_histogram: prints the power histogram to console (called from main.c).

Code review / suggestions:
- ISR safety: avoid calling non-ISR-safe functions in the ISR. Currently get_absolute_time() and LED
    helpers are used; confirm these are ISR-safe on your platform. Prefer capturing minimal state and
    deferring heavier work to the main loop.
- Atomicity / ordering: shared queue variables (`queue_head_sband`, `queue_tail_sband`, `queue_count_sband`)
    are updated in both ISR and main code. Consider using interrupt-disable sections or atomic primitives
    when updating multi-field state to avoid races on preemptive cores/optimizers.
- Bounds checks: when writing `queue_len`, `snr`, and `rssi` into `packet_queue_sband[idx]`, validate
    `payload_len` before writing to offsets derived from `offsetof(sband_payload_t, ...)` to avoid corrupting
    memory for unexpectedly short packets.
- Encapsulation: storing metadata inside a raw uint8_t buffer works, but consider a small struct wrapper
    (e.g., `struct { uint8_t data[...]; uint8_t len; int8_t snr; int8_t rssi; }`) to make intent clearer
    and reduce reliance on `offsetof` and manual offsets.
- Histogram type and saturation: `power_histogram_sband` is `int`; prefer `uint32_t` and optionally
    saturate increments to avoid overflow for very long runs.
- Linker visibility: `sband_print_histogram` must be non-static and declared in the header (`doSband.h`)
    to avoid undefined references; ensure the header prototype matches and this C file is compiled into
    the target.
- Logging and asserts: add defensive checks (e.g., assert payload lengths and queue bounds) behind
    an `#ifdef DEBUG` so they can aid development without incurring runtime cost in release builds.
- Code comments: document the wire-format offsets (preamble removal by SX1280) and the meaning of
    `queue_len` and embedded metadata for future maintainers.

Minor style:
- Keep ISR small and avoid printf; prefer ring-buffer or setting an atomic flag for main loop processing.
- Use consistent types for signed/unsigned (e.g., `rssi` as int8_t vs stored into uint8_t buffer)
    and comment any intentional reinterpretation.

End review.
*/

// Packet queue structure definitions (for ISR access)
#define QUEUE_SIZE_SBAND 15
#define PACKET_SIZE_SBAND 256
// Payload format placed after 4-byte preamble on the wire.
// SX1280 strips the 4-byte preamble on receive, so received payload starts at offset 0.
typedef struct __attribute__((packed)) sband_payload_t {
    uint8_t dest;              // network: destination
    uint8_t src;               // network: source
    uint8_t ack;               // network: ack flag
    uint8_t serial;            // network: serial
    uint8_t queue_len;         // stored by ISR/polling: length of received packet
    int8_t  power;             // transmit power
    uint32_t packet_set_count; // set counter (little-endian)
    int8_t  snr;               // receiver fills this
    int8_t  rssi;              // receiver fills this
    uint16_t histogram[30];    // optional histogram data (30 bins)
    uint8_t  pad[174];         // padding to make sizeof(sband_payload_t)==246 bytes
} sband_payload_t;

// Packet queue storage (arrays) — packet_sband_t removed as requested
volatile uint8_t packet_queue_sband[QUEUE_SIZE_SBAND][PACKET_SIZE_SBAND];

// Global queue variables for ISR access
volatile uint8_t queue_head_sband = 0;  // ISR writes here
volatile uint8_t queue_tail_sband = 0;  // Main loop reads here
volatile uint8_t queue_count_sband = 0;
volatile absolute_time_t sband_last_rx_time;  // For LED blue timing

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

// State machine variables for non-blocking TX
// Note: Sband_Transmitting is controlled by global UHF_TX state (!UHF_TX = Sband RX)
static int current_tx_power_sband = SBAND_MAX_POWER;  // Start at max power, count down to min
volatile absolute_time_t sband_last_rx_time = 0;    // Timestamp of last packet (for LED logic)
static bool radio_initialized = false;              // sband radio initialized flag (true = initialized, false = not initialized)

// Per-transmit serial number (1-byte), starts at 0 and increments on each TX
static uint8_t SerNo = 0;

// RX mode selection: Set to 1 for interrupt-driven, 0 for polling
#define SBAND_RX_USE_INTERRUPTS 0  // Change to 1 to use DIO1 interrupts

// Timing configuration
const uint32_t interval_Sband = 1000000;  // 1 second between operations

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
        sband_last_rx_time = get_absolute_time();
        // Check if queue has space
        if (queue_count_sband < QUEUE_SIZE_SBAND) {
            uint8_t idx = queue_head_sband;

            // Read packet from FIFO directly into our array slot -- even if there is a CRC error on this packet
            // datasheet page 49: It is important to note that all the received data will be written to the data 
            // buffer even if the CRC is invalid, permitting user-defined post processing of corrupted data.
            int payload_len = sband_packet_from_fifo((uint8_t*)packet_queue_sband[idx]);
            if (payload_len > 0) {
                // Store length, SNR and RSSI inside the received buffer so the
                // main/core1 processing code can extract them without separate arrays.
                packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)] = (uint8_t)payload_len;
                int8_t snr = sband_get_snr();
                int16_t rssi = sband_get_rssi();
                packet_queue_sband[idx][offsetof(sband_payload_t, snr)] = (uint8_t)snr;
                packet_queue_sband[idx][offsetof(sband_payload_t, rssi)] = (uint8_t)rssi;
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

#define TRY_INIT (52)  // Maximum initialization attempts

/*
 * Attempt to initialize SBand radio with retry logic
 * Includes TX validation tests
 * Returns: true if initialization successful, false if all retries exhausted
 */
bool attempt_sband_init(void) {
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

        if (ip == 0) {
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

        if (ip == 0) {
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
 * Main SBand operation loop
 */

// We want to get something done every time we're called but we don't want
// to hog the CPU cycles.  Here's the flowchart of the logic:
//
// If there is a received packet waiting in the radio, maybe because the interrupt
//       isn't working, bring it to the queue and return
// If it's too early, return and let UHF do its thing
// If the radio hasn't been initialized, try to initialize it and return. 
// If UHF just started transmitting, set up RX irqs and enter RX mode and return
// If UHF just stopped transmitting, set up TX mode and disable RX irqs and return
// if UHF is transmitting, process the RX queue into the histogram and return
// If UHF is not transmitting, send one packet at the current power level and return

void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX) {
    // called functions return true if they did significant work and we should release the CPU

    static sband_payload_t payload;
    static absolute_time_t sband_last_time = {0};
    static bool prev_UHF_TX = true;  // Start assuming UHF is TX (Sband is RX)

    if (sband_poll_for_rx_packet(buffer_Sband_RX)) { return; }  // Check for RX packet if polling mode

    if (absolute_time_diff_us(sband_last_time, get_absolute_time()) < interval_Sband) {
        return;  // Too often for more processing
    }
    sband_last_time = get_absolute_time();

    if (!radio_initialized) {
        radio_initialized = attempt_sband_init();
        return;  // Return after initialization attempt
    }

    //Everything after this is only executed if the radio is initialized

    if (UHF_TX != prev_UHF_TX) {    // Execute sband state change which follows UHF
        printf("SBand: UHF_TX changed %d->%d, radio_init=%d\n",
            prev_UHF_TX, UHF_TX, radio_initialized);
        prev_UHF_TX = UHF_TX;
        if (UHF_TX) {  
            printf("UHF_TX went from false to true: Stop Sband TX, start Sband RX\n");
            if (sband_begin_rx(buffer_Sband_RX)) { return;}  // Enter RX mode
           } else {  
            printf("UHF_TX went from true to false: Stop Sband RX, start Sband TX\n");
            if (sband_begin_tx(buffer_Sband_TX)) { return; }  // Enter TX mode
           }
    }

    // UHF is transmitting, so SBand is receiving.  Handle continuing RX processing
    if (UHF_TX) {
        if (sband_process_queue(buffer_Sband_RX)) { return; }  // Drain RX queue into histogram
    } else {  
        if (sband_send_one_packet(buffer_Sband_TX)) { return; }  // Transmit one packet at current power level
    }
printf("SBand: Empty RX queue\n");
}   

bool sband_poll_for_rx_packet() {
//  check for any packets in the radio FIFO.  If the interrupt is working, we are in transmit mode,
//  or the radio isn't initialized, we shouldn't see any
    //printf("SBand: Polling for RX packet: radio_initialized=%d, UHF_TX=%d\n", radio_initialized, UHF_TX);
    if (radio_initialized && UHF_TX && sband_rx_done()) {
        printf("SBand: RX_DONE detected via polling\n");
        blue();  // Show packet arrival
        sband_last_rx_time = get_absolute_time();

        // Check if queue has space
        if (queue_count_sband < QUEUE_SIZE_SBAND) {
            uint8_t idx = queue_head_sband;

                // Read packet from FIFO into the array slot
                int payload_len = sband_packet_from_fifo((uint8_t*)packet_queue_sband[idx]);
                if (payload_len > 0) {
                    // Store length, SNR and RSSI inside the packet buffer
                    packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)] = (uint8_t)payload_len;
                    int8_t snr = sband_get_snr();
                    int16_t rssi = sband_get_rssi();
                    packet_queue_sband[idx][offsetof(sband_payload_t, snr)] = (uint8_t)snr;
                    packet_queue_sband[idx][offsetof(sband_payload_t, rssi)] = (uint8_t)rssi;
                }
                printf("SBand: Packet received, len=%d, SNR=%d, RSSI=%d\n",
                        packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)],
                        (int8_t)packet_queue_sband[idx][offsetof(sband_payload_t, snr)],
                        (int8_t)packet_queue_sband[idx][offsetof(sband_payload_t, rssi)]);
                
                // When modifying queue_head_sband and queue_count_sband together from ISR/main, disable DIO1 IRQ briefly:
                uint32_t save = save_and_disable_interrupts();
                queue_head_sband = (queue_head_sband + 1) % QUEUE_SIZE_SBAND;  // circular buffer
                queue_count_sband++;
                restore_interrupts(save);
        }

        // Clear RX_DONE and continue listening
        sband_clear_irq_status(SX1280_IRQ_RX_DONE);
        sband_listen();
        return true;  // Indicate that we handled an RX packet
    }  // end of RX polling & enqueueing
    return false;  // No RX packet handled
}

bool sband_begin_rx(char *buffer_Sband_RX) {
    (void)buffer_Sband_RX; // unused currently
    printf("SBand: Transitioning to RX mode driven by UHF transmitting\n");
        sband_listen();
        gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, true);
        buffer_Sband_RX[0] = '\0';      //  Clear buffer_Sband_TX to avoid stale messages to user
        return true;
    }

bool sband_begin_tx(char *buffer_Sband_TX) {
    (void)buffer_Sband_TX; // unused currently
    printf("SBand: Transitioning to TX mode driven by UHF stopping transmission\n");
        gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, false);
        sband_set_mode(SX1280_MODE_STDBY_RC);
        current_tx_power_sband = SBAND_MAX_POWER;  // Start at max power
        return true;
    }

bool sband_process_queue(char *buffer_Sband_RX) {
    (void)buffer_Sband_RX; // unused in this implementation

    if (queue_count_sband == 0) {
        return false;  // No packets to process
    }   

    // Process all packets in receive queue
    while (queue_count_sband > 0) {
        uint8_t idx = queue_tail_sband;
        blue();
        // Parse structured payload sent by peer
        int power = 0;
        uint8_t stored_len = packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)];
        if (stored_len <= PACKET_SIZE_SBAND) {
            sband_payload_t rpl;
            // Received payload starts at packet_queue_sband[idx][0].  Make a local copy so the
            // ISR can manage the queue while we process this packet.
            size_t to_copy = stored_len < sizeof(rpl) ? stored_len : sizeof(rpl);
            memset(&rpl, 0, sizeof(rpl));
            memcpy(&rpl, (const void *)packet_queue_sband[idx], to_copy);
            power = (int)rpl.power;

            // Initialize offset on first valid packet
            if (!sband_offset_initialized) {
                sband_rx_packet_set_offset = rpl.packet_set_count - sband_packet_set_count;
                sband_offset_initialized = true;
                printf("SBand: RX packet set count offset initialized to %u\n",
                       sband_rx_packet_set_offset);
            }
// Pico2's sdk is little endian. This was stored with little endian.
            uint32_t adjusted_count = rpl.packet_set_count - sband_rx_packet_set_offset;

            // Update last received packet set count
            if (adjusted_count > sband_last_rx_packet_set_count) {
                sband_last_rx_packet_set_count = adjusted_count;
            }

            // Update power histogram
            if (power >= -18 && power <= 13) {
                power_histogram_sband[power + 18]++;
            }
        }

        // Update queue.  First, disable interrupts briefly to avoid races
        uint32_t save = save_and_disable_interrupts();
        queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;  // circular buffer
        queue_count_sband--;
        restore_interrupts(save);
    }

// Calculate and display packet loss statistics (only when count changes)
        if ((sband_last_rx_packet_set_count > 0 &&
            sband_last_rx_packet_set_count != sband_last_printed_packet_set_count)) {
            printf("\n=== SBand Packet Loss Statistics (Set #%lu) ===\n", sband_last_rx_packet_set_count);
            printf("Power(dBm) | Expected | Received | Lost | Loss%%\n");
            printf("-----------|----------|----------|------|--------\n");

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

                printf("%10d | %8lu | %8d | %4lu | %5.1f%%\n",
                       power_dbm, expected, received, lost, loss_percent);
            }
            printf("====================================================\n\n");

            // Update last printed count
            sband_last_printed_packet_set_count = sband_last_rx_packet_set_count;
        } // End of packet loss statistics display


    return true;  // Indicate that we processed the RX queue
}

bool sband_send_one_packet(char *buffer_Sband_TX) {
    if (!UHF_TX && radio_initialized) {         // this should always be true if we were called
        sband_set_tx_params(current_tx_power_sband, 0x02);  // Set power, ramp 20μs

        // Create packet: packed sband_payload_t
        sband_payload_t payload;
        memset(&payload, 0, sizeof(payload));
        // Network bytes (this is not the preamble that SX1280 adds/removes automatically)
        payload.dest = 0xFF; // broadcast by default
        payload.src = 0xDD;  // source id 221
        payload.ack = 0x00; // no ack requested
        payload.serial = SerNo++;  // per-packet serial number and increment (wraps naturally at 255->0)
        payload.power = (int8_t)current_tx_power_sband;
        payload.packet_set_count = sband_packet_set_count;
        payload.snr = 0;
        payload.rssi = 0;
        // Send our histogram for logging by a monitor receiver (neither the ground station nor the satellite)
        for (int i = 0; i < 30; i++) {
            payload.histogram[i] = (uint16_t)power_histogram_sband[i];
        }

        // Fill padding with sequential values 0x00,0x01,... to reach full payload size
        for (size_t i = 0; i < sizeof(payload.pad); i++) {
            payload.pad[i] = (uint8_t)(i & 0xFF);
        }
        // Send payload directly from stack (send exact struct size)
        sband_clear_irq_status(0xFFFF);
        sband_packet_to_fifo((uint8_t *)&payload, sizeof(payload));
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
            sleep_us(100);
        }
        if (timeout <= 0) {
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
    
        // If our "packet received" LED color is stale, switch back to indicating UHF TX only.
        if (UHF_TX && absolute_time_diff_us(sband_last_rx_time, get_absolute_time()) > 2000000) {   red();    }
        return true;  // Indicate that we sent a packet
        }
    printf("Sband: Warning: Not sending packet: UHF_TX=%d, radio_initialized=%d\n", UHF_TX, radio_initialized);
    return false;  // We sent no packet, took no time
}

/* 
// 
    // Handle UHF_TX state changes FIRST (must happen even if radio not initialized)
    // This allows re-initialization attempts when transitioning to TX mode
    static bool prev_UHF_TX = true;  // Start assuming UHF is TX (Sband is RX)

    if (UHF_TX != prev_UHF_TX) {    // UHF_TX state changed -- We follow.  This is the initialization/switchover code.
        printf("SBand: UHF_TX changed %d->%d, radio_init=%d\n",
               prev_UHF_TX, UHF_TX, radio_initialized);
        if (UHF_TX) {
            // UHF_TX went from false to true: Stop Sband TX, start Sband RX
            printf("SBand: Transitioning to RX mode driven by UHF transmitting\n");
            if (radio_initialized) {
                sband_listen();
                gpio_set_irq_enabled(SAMWISE_SBAND_D1_PIN, GPIO_IRQ_EDGE_RISE, true);
            } else {
                printf("SBand: Cannot enter RX - radio not initialized!\n");
            }
            buffer_Sband_TX[0] = '\0';  // Clear TX buffer
        } else {
            // UHF_TX went from true to false: Stop Sband RX, start Sband TX
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
    }   // End of UHF_TX state change handling

#if !SBAND_RX_USE_INTERRUPTS
    // We're in RX, but DIO1 is broken, so use polling mode: Check for RX_DONE every loop iteration
    if (radio_initialized && UHF_TX) {
        if (sband_rx_done()) {
            printf("SBand: RX_DONE detected via polling\n");
            blue();  // Show packet arrival
            sband_last_rx_time = get_absolute_time();

            // Check if queue has space
            if (queue_count_sband < QUEUE_SIZE_SBAND) {
                uint8_t idx = queue_head_sband;

                    // Read packet from FIFO into the array slot
                    int payload_len = sband_packet_from_fifo((uint8_t*)packet_queue_sband[idx]);
                    if (payload_len > 0) {
                        // Store length, SNR and RSSI inside the packet buffer
                        packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)] = (uint8_t)payload_len;
                        int8_t snr = sband_get_snr();
                        int16_t rssi = sband_get_rssi();
                        packet_queue_sband[idx][offsetof(sband_payload_t, snr)] = (uint8_t)snr;
                        packet_queue_sband[idx][offsetof(sband_payload_t, rssi)] = (uint8_t)rssi;
                    }
                    printf("SBand: Packet received, len=%d, SNR=%d, RSSI=%d\n",
                           packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)],
                           (int8_t)packet_queue_sband[idx][offsetof(sband_payload_t, snr)],
                           (int8_t)packet_queue_sband[idx][offsetof(sband_payload_t, rssi)]);
                    
                   // Update queue
                    queue_head_sband = (queue_head_sband + 1) % QUEUE_SIZE_SBAND;  // circular buffer
                    queue_count_sband++;
            }

            // Clear RX_DONE and continue listening
            sband_clear_irq_status(SX1280_IRQ_RX_DONE);
            sband_listen();
        }
    }  // end of RX polling & enqueueing
#endif

    // Skip all radio operations if not initialized PHM might not need this as queue will be empty?
    if (!radio_initialized) {return;}

    // SBAND_RX: Ocasionally process packet queue filled by ISR or polling only when UHF is transmitting
    if (UHF_TX && radio_initialized &&
        absolute_time_diff_us(previous_time_Sband_RX, get_absolute_time()) >= interval_Sband_RX) {
        previous_time_Sband_RX = get_absolute_time();
//        sprintf(buffer_Sband_RX, "SRX ");
        // Process all packets in receive queue
        while (queue_count_sband > 0) {
            uint8_t idx = queue_tail_sband;
            blue();
            // Parse structured payload sent by peer
            int power = 0;
            uint8_t stored_len = packet_queue_sband[idx][offsetof(sband_payload_t, queue_len)];
            if (stored_len <= PACKET_SIZE_SBAND) {
                sband_payload_t rpl;
                // Received payload starts at packet_queue_sband[idx][0].  Make a local copy so the
                // ISR can manage the queue while we process this packet.
                memcpy(&rpl, (const void*)packet_queue_sband[idx], sizeof(rpl));
                power = (int)rpl.power;
                uint32_t rx_packet_set_count = rpl.packet_set_count;
                // Packet enqueued and data extracted successfully - print out the details
                printf("SBand: RX Packet - len=%u, serial number=%d, power=%d dBm, SNR=%d dB, RSSI=%d dBm, set_count=%lu\n",
                       (unsigned)stored_len, rpl.serial, power, (int8_t)rpl.snr, (int8_t)rpl.rssi, rx_packet_set_count);
                // print using helper
                sband_print_payload(&rpl);
                sband_last_rx_packet_set_count = rx_packet_set_count;
                // Histogram begins recording after both RX and TX have started
                // to avoid mis-alignment due to late starts
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
                // SBand receives from another identical SBand radio (same band)
                int hist_index = power - SBAND_MIN_POWER;  // Map SBand min power to index 0
                if (hist_index >= 0 && hist_index < 32) {
                    // NOTE: This increment happens on Core1 while Core0 may read
                    // the histogram for realtime display. At rare moments when the
                    // higher byte of the counter wraps and the increment carries
                    // into that higher byte, the displayed histogram value may
                    // briefly be off by 256. This is acceptable for a realtime
                    // display and avoids locking overhead in the hot path.
                    power_histogram_sband[hist_index]++;
                } else {
                    printf("SBand Warning: Received out-of-range power value: %d (expected SBand range %d-%d)\n",
                        power, SBAND_MIN_POWER, SBAND_MAX_POWER);
                }
                // Update queue
                queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;
                queue_count_sband--;
            } else if (stored_len == 0) {
                // Empty packet — discard and log
                printf("SBand Error: Received zero-length packet at queue index %u\n", idx);
                queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;
                queue_count_sband--;
            } else if (stored_len < (int)sizeof(sband_payload_t)) {
                // Packet too small to contain expected structured payload
                printf("SBand Error: Received undersized packet (len=%d, expected>=%zu) at index %u\n",
                       (int)stored_len, sizeof(sband_payload_t), idx);
                queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;
                queue_count_sband--;
            } else {
                // stored_len is larger than buffer size or unexpected — discard and log
                printf("SBand Error: Received oversized packet (len=%d) at index %u, max=%d\n",
                       (int)stored_len, idx, (int)PACKET_SIZE_SBAND);
                queue_tail_sband = (queue_tail_sband + 1) % QUEUE_SIZE_SBAND;
                queue_count_sband--;
            } // End of stored_len handling
        }  // End of while (queue_count_sband > 0)
        // Calculate and display packet loss statistics (only when count changes)
        if ((sband_last_rx_packet_set_count > 0 &&
            sband_last_rx_packet_set_count != sband_last_printed_packet_set_count)) {
            printf("\n=== SBand Packet Loss Statistics (Set #%lu) ===\n", sband_last_rx_packet_set_count);
            printf("Power(dBm) | Expected | Received | Lost | Loss%%\n");
            printf("-----------|----------|----------|------|--------\n");

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

                printf("%10d | %8lu | %8d | %4lu | %5.1f%%\n",
                       power_dbm, expected, received, lost, loss_percent);
            }
            printf("====================================================\n\n");

            // Update last printed count
            sband_last_printed_packet_set_count = sband_last_rx_packet_set_count;
        } // End of packet loss statistics display
    }  // End of RX processing

    // SBAND_TX: Transmit when UHF is in RX mode (!UHF_TX)
    // If currently transmitting (UHF in RX mode), send one packet per invocation
    if (!UHF_TX && radio_initialized) {
        sband_set_tx_params(current_tx_power_sband, 0x02);  // Set power, ramp 20μs

        // Create packet: packed sband_payload_t
        sband_payload_t payload;
        memset(&payload, 0, sizeof(payload));
        // Network bytes (this is not the preamble that SX1280 adds/removes automatically)
        payload.dest = 0xFF; // broadcast by default
        payload.src = 0xDD;  // source id 221
        payload.ack = 0x00; // no ack requested
        payload.serial = SerNo++;  // per-packet serial number and increment (wraps naturally at 255->0)
        payload.power = (int8_t)current_tx_power_sband;
        payload.packet_set_count = sband_packet_set_count;
        payload.snr = 0;
        payload.rssi = 0;
        // Send our histogram for logging by a monitor receiver (neither the ground station nor the satellite)
        for (int i = 0; i < 30; i++) {
            payload.histogram[i] = (uint16_t)power_histogram_sband[i];
        }

        // Fill padding with sequential values 0x00,0x01,... to reach full payload size
        for (size_t i = 0; i < sizeof(payload.pad); i++) {
            payload.pad[i] = (uint8_t)(i & 0xFF);
        }
        // Send payload directly from stack (send exact struct size)
        sband_clear_irq_status(0xFFFF);
        sband_packet_to_fifo((uint8_t *)&payload, sizeof(payload));
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
            sleep_us(100);
        }
        if (timeout <= 0) {
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
*/

// Print SBand power histogram to console (callable from main.c)
void sband_print_histogram(void) {
    printf("SRX ");

    if (nCRC_sband > 0) {
        printf("%d CRC ", nCRC_sband);
        nCRC_sband = 0;  // Zero out CRC error count after displaying  PHM this is probably not working right
    }

    for (int i = 0; i < 32; i++) {
        printf("%d ", power_histogram_sband[i]);
    }
    printf("\n");
}

// Pretty-print a received or to-be-sent `sband_payload_t` storage block.
void sband_print_payload(const sband_payload_t *p) {
    if (!p || p->queue_len == 0) {
        printf("SBand: payload=NULL\n");
        return;
    }

    printf("SBand Payload:\n");
    printf("  dest=%u src=%u ack=%u serial=%u len=%u\n",
           (unsigned)p->dest, (unsigned)p->src, (unsigned)p->ack, (unsigned)p->serial, (unsigned)p->queue_len);
    printf("  power=%d dBm, set_count=%lu, SNR=%d dB, RSSI=%d dBm\n",
           (int)p->power, (unsigned long)p->packet_set_count, (int8_t)p->snr, (int8_t)p->rssi);

    // Print histogram (30 bins)
    printf("  histogram:");
    for (int i = 0; i < 30; ++i) {
        printf(" %u", (unsigned)p->histogram[i]);
    }
    printf("\n");
}
