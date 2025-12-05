#pragma once
#include <stdint.h>
#include "pico/time.h"
#include "pins.h"

// Initialize UHF radio (includes ISR setup and tx_done test)
void initUHF(spi_pins_t *spi_pins);

// Main UHF operation loop (RX/TX timing)
void doUHF(char *buffer_RADIO_RX, char *buffer_RADIO_TX);

// ISR handler (called by GPIO interrupt)
void dio0_isr(uint gpio, uint32_t events);

// LED color contribution from UHF (for additive color mixing)
extern uint8_t uhf_led_r;
extern uint8_t uhf_led_g;
extern uint8_t uhf_led_b;
