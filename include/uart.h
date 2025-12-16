#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// Function prototypes
void on_uart_rx(void);  // I think this should not be here
uint8_t init_uart(void);
