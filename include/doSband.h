#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "pins.h"

// S-band Radio Power Configuration
#define SBAND_MAX_POWER 13        // Maximum TX power in dBm (SX1280 hardware limit)
#define SBAND_MIN_POWER 5         // Minimum TX power in dBm

// Initialize SBand radio (includes ISR setup and tx_done test)
bool sband_poll_for_rx_packet();
bool attempt_sband_init(void);
bool sband_begin_rx(char *buffer_Sband_RX);
bool sband_begin_tx(char *buffer_Sband_TX);
bool sband_process_queue(char *buffer_Sband_RX);
bool sband_send_one_packet(char *buffer_Sband_TX);
extern void sband_print_histogram(void);

// Main SBand operation loop (RX/TX timing)
void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX);

// ISR handler (called by GPIO interrupt)
void sband_dio1_isr(uint gpio, uint32_t events);

// Print the SBand power histogram to console
void sband_print_histogram(void);

// Forward-declare payload struct and print helper
typedef struct sband_payload_t sband_payload_t;
void sband_print_payload(const sband_payload_t *p);

// Timing variables
extern const uint32_t interval_Sband;  // Changed from static to extern
