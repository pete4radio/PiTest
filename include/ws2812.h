#pragma once
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// LED function declarations
int ws2812_init();
void red();
void green();
void white();

