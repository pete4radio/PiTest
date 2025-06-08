#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "uart.h"
#include "main_gps_uart_shared_buffer.h"

static int chars_rxed = 0;

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        // If we have received a newline character, stop reading, terminate and return
        if (ch == '\n' || ch == '\r') {
            if (chars_rxed < BUFLEN) {
                buffer_UART[chars_rxed] = '\0'; // null terminate the string
            }
            chars_rxed = 0; // reset for the next line
            return; // exit the handler
        //  if the buffer still has the last string, drop the new one
        } else if (buffer_UART[0] != '\0') {
            return;
        }
        // put it into buffer_UART
        if (chars_rxed < BUFLEN - 1) { // leave space for null terminator
            buffer_UART[chars_rxed] = ch;
            chars_rxed++;
            buffer_UART[chars_rxed] = '\0'; // null terminate the string
            chars_rxed++;
        } 
    }
}

uint8_t init_uart() {
    // Initialize the UART, and set up the pins
    
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Set the FIFO to be 16 bytes deep, and clear it
    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");

return PICO_OK;
}