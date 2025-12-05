#pragma once
#include <stdint.h>
#include "pico/time.h"
#include "pins.h"

// Initialize SBand radio (includes ISR setup and tx_done test)
void initSband(spi_pins_t *spi_pins);

// Main SBand operation loop (RX/TX timing)
void doSband(char *buffer_Sband_RX, char *buffer_Sband_TX);

// ISR handler (called by GPIO interrupt)
void sband_dio0_isr(uint gpio, uint32_t events);

// LED color contribution from SBand (for additive color mixing)
extern uint8_t sband_led_r;
extern uint8_t sband_led_g;
extern uint8_t sband_led_b;
