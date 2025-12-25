#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "pins.h"

// Initialize SBand radio (includes ISR setup and tx_done test)
void initSband(spi_pins_t *spi_pins);

// Main SBand operation loop (RX/TX timing)
void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX, spi_pins_t *spi_pins);

// ISR handler (called by GPIO interrupt)
void sband_dio1_isr(uint gpio, uint32_t events);

// Timestamp of last SBand packet received (for LED logic)
extern volatile absolute_time_t sband_last_rx_time;

// LED color contribution from SBand (for additive color mixing)
extern volatile uint8_t sband_led_r;
extern volatile uint8_t sband_led_g;
extern volatile uint8_t sband_led_b;
