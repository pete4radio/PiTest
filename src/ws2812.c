/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "pins.h"

PIO global_pio;
uint global_sm;

/**
 * NOTE:
 *  Take into consideration if your WS2812 is a RGB or RGBW variant.
 *
 *  If it is RGBW, you need to set IS_RGBW to true and provide 4 bytes per 
 *  pixel (Red, Green, Blue, White) and use urgbw_u32().
 *
 *  If it is RGB, set IS_RGBW to false and provide 3 bytes per pixel (Red,
 *  Green, Blue) and use urgb_u32().
 *
 *  When RGBW is used with urgb_u32(), the White channel will be ignored (off).
 *
 */
#define IS_RGBW false
//really only one
#define NUM_PIXELS 2

// Check the pin is compatible with the platform
#if SAMWISE_NEOPIXEL_PIN >= NUM_BANK0_GPIOS
#error Attempting to use a pin>=32 on a platform that does not support it
#endif

static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            ((uint32_t) (w) << 24) |
            (uint32_t) (b);
}

// TX
void red(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0x10, 0, 0));
    }
}

//RX
void green(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0, 0x08, 0));
    }
}

// Idle indicated by dim white
void white(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0x01, 0x01, 0x01));
    }
}

// UHF Listening - Yellow (Red + Green)
void yellow(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0x10, 0x08, 0));
    }
}

// SBand Listening - Cyan (Green + Blue)
void cyan(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0, 0x08, 0x10));
    }
}

// SBand Receiving - Magenta (Red + Blue)
void magenta(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0x10, 0, 0x10));
    }
}

// SBand Transmitting - Blue
void blue(){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(0, 0, 0x10));
    }
}

// Set LED to specific RGB color (for additive color mixing)
void set_led_color(uint8_t r, uint8_t g, uint8_t b){
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, urgb_u32(r, g, b));
    }
}


int ws2812_init() {
    //set_sys_clock_48();
    printf("WS2812_init: Smoke Test, using pin %d\n", SAMWISE_NEOPIXEL_PIN);

    // todo get free sm
    PIO pio;
    uint sm;
    uint offset;

    


// Check the pin is compatible with the platform
#if SAMWISE_NEOPIXEL_PIN >= NUM_BANK0_GPIOS
#error Attempting to use a pin>=32 on a platform that does not support it
#endif


    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, SAMWISE_NEOPIXEL_PIN, 1, true);
    hard_assert(success);

    ws2812_program_init(pio, sm, offset, SAMWISE_NEOPIXEL_PIN, 800000, IS_RGBW);   
    global_pio = pio;
    global_sm = sm;

    return PICO_OK;
}