#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "ws2812.pio.h"
#include "pins.h"

PIO global_pio;
uint global_sm;

#define IS_RGBW false
#define NUM_PIXELS 1
#define TIME_ON_MS 75  // How long each color stays visible

// Color queue structure
#define COLOR_QUEUE_SIZE 8
typedef struct {
    uint32_t color_grb;
    bool valid;
} color_request_t;

static color_request_t color_queue[COLOR_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;  // Where we write new requests
static volatile uint8_t queue_tail = 0;  // Where we read for display
static volatile uint32_t current_color = 0;  // Currently displayed color
static struct repeating_timer color_timer;

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

// Non-blocking function to update the LED strip with a color
static void update_led_immediate(uint32_t color_grb) {
    for (int i = 0; i < NUM_PIXELS; ++i) {
        put_pixel(global_pio, global_sm, color_grb);
    }
    current_color = color_grb;
}

// Timer callback - checks queue every TIME_ON_MS
static bool color_timer_callback(struct repeating_timer *t) {
    // Check if there's a new color request in the queue
    if (queue_tail != queue_head && color_queue[queue_tail].valid) {
        // Display the next color in queue
        uint32_t new_color = color_queue[queue_tail].color_grb;
        color_queue[queue_tail].valid = false;
        
        // Move to next queue entry
        queue_tail = (queue_tail + 1) % COLOR_QUEUE_SIZE;
        
        // Update the LED strip
        update_led_immediate(new_color);
    }
    // If queue is empty, keep current color (do nothing)
    
    return true;  // Keep timer running
}

// Enqueue a color change request (non-blocking)
static void enqueue_color(uint32_t color_grb) {
    uint8_t next_head = (queue_head + 1) % COLOR_QUEUE_SIZE;
    
    // Check if queue is full
    if (next_head == queue_tail) {
        // Queue full - drop oldest request by moving tail forward
        queue_tail = (queue_tail + 1) % COLOR_QUEUE_SIZE;
    }
    
    // Add new color to queue
    color_queue[queue_head].color_grb = color_grb;
    color_queue[queue_head].valid = true;
    queue_head = next_head;
}

// Public color functions - now non-blocking

// TX
void red(){
    enqueue_color(urgb_u32(0x10, 0, 0));
}

//RX
void green(){
    enqueue_color(urgb_u32(0, 0x08, 0));
}

// Idle indicated by dim white
void white(){
    enqueue_color(urgb_u32(0x01, 0x01, 0x01));
}

// UHF Listening - Yellow (Red + Green)
void yellow(){
    enqueue_color(urgb_u32(0x10, 0x08, 0));
}

// SBand Listening - Cyan (Green + Blue)
void cyan(){
    enqueue_color(urgb_u32(0, 0x08, 0x10));
}

// SBand Receiving - Magenta (Red + Blue)
void magenta(){
    enqueue_color(urgb_u32(0x10, 0, 0x10));
}

// SBand Transmitting - Blue
void blue(){
    enqueue_color(urgb_u32(0, 0, 0x10));
}

// Set LED to specific RGB color (for additive color mixing)
void set_led_color(uint8_t r, uint8_t g, uint8_t b){
    enqueue_color(urgb_u32(r, g, b));
}

int ws2812_init() {
    printf("WS2812_init: Smoke Test, using pin %d\n", SAMWISE_NEOPIXEL_PIN);

    PIO pio;
    uint sm;
    uint offset;

    // Initialize queue
    for (int i = 0; i < COLOR_QUEUE_SIZE; i++) {
        color_queue[i].valid = false;
        color_queue[i].color_grb = 0;
    }
    queue_head = 0;
    queue_tail = 0;

    // This will find a free pio and state machine for our program and load it for us
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, SAMWISE_NEOPIXEL_PIN, 1, true);
    hard_assert(success);

    ws2812_program_init(pio, sm, offset, SAMWISE_NEOPIXEL_PIN, 800000, IS_RGBW);   
    global_pio = pio;
    global_sm = sm;

    // Set initial color to white (idle)
    current_color = urgb_u32(0x01, 0x01, 0x01);
    update_led_immediate(current_color);

    // Start timer callback (runs every TIME_ON_MS)
    // Negative delay = milliseconds
    add_repeating_timer_ms(TIME_ON_MS, color_timer_callback, NULL, &color_timer);
    
    printf("WS2812: Timer started (checking every %d ms)\n", TIME_ON_MS);

    return PICO_OK;
}