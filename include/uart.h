#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// For the ADCS board, Pins 1 & 2 are GPIO 2 and 3, and are UART0 and function 11

#define UART_TX_PIN 2
#define UART_RX_PIN 3

// Function prototypes
void on_uart_rx(void);  // I think this should not be here
uint8_t init_uart(void);
