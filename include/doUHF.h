#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "pins.h"

// UHF Radio Power Configuration
#define UHF_MAX_POWER 14          // Maximum TX power in dBm
#define UHF_MIN_POWER 2           // Minimum TX power in dBm
#define UHF_TIME_ON_THE_AIR 1100  // Time per transmission in milliseconds

// Initialize UHF radio (includes ISR setup and tx_done test)
void initUHF(void);

// Main UHF operation loop (RX/TX timing)
void doUHF(char *buffer_RADIO_RX, char *buffer_RADIO_TX);

// ISR handler (called by GPIO interrupt)
void dio0_isr(uint gpio, uint32_t events);

// Global state: true = UHF TX (SBand RX), false = UHF RX (SBand TX)
extern volatile bool UHF_TX;

// Timestamp of last UHF packet received (for LED logic)
extern volatile absolute_time_t uhf_last_rx_time;

// LED color contribution from UHF (for additive color mixing)
extern volatile uint8_t uhf_led_r;
extern volatile uint8_t uhf_led_g;
extern volatile uint8_t uhf_led_b;
