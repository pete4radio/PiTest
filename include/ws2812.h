#pragma once
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include <stdint.h>

// LED function declarations
int ws2812_init();
void red();
void green();
void white();
void yellow();
void cyan();
void magenta();
void blue();

// Set LED to specific RGB color (additive for showing multiple states)
void set_led_color(uint8_t r, uint8_t g, uint8_t b);

