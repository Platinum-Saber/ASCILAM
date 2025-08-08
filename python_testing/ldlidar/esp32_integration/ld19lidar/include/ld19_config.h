#ifndef LD19_CONFIG_H
#define LD19_CONFIG_H

// Hardware configuration
#define LD19_UART_NUM       2
#define LD19_RX_PIN         16
#define LD19_TX_PIN         17
#define LD19_BAUD_RATE      230400

// Protocol constants
#define LD19_HEADER_BYTE    0x54
#define LD19_VERLEN_BYTE    0x2C
#define LD19_FRAME_SIZE     47
#define LD19_POINTS_PER_FRAME 12

// Network configuration defaults
#define DEFAULT_UDP_PORT    5005
#define DEFAULT_PC_IP       "192.168.115.150"

// Timeout settings
#define SERIAL_READ_TIMEOUT_MS  25
#define UDP_READ_TIMEOUT_MS     20
#define WIFI_CONNECT_TIMEOUT_S  30

// Debug settings
#define DEBUG_SERIAL_BAUD   115200
#define DEBUG_ENABLED       true

// Mode definitions for easy switching
typedef enum {
    LD19_MODE_RAW_HEX = 0,
    LD19_MODE_PARSED_FRAMES,
    LD19_MODE_UDP_FORWARD,
    LD19_MODE_COMBO_PARSE_UDP
} ld19_mode_t;

// Error codes
typedef enum {
    LD19_OK = 0,
    LD19_ERROR_SERIAL_INIT,
    LD19_ERROR_WIFI_CONNECT,
    LD19_ERROR_UDP_INIT,
    LD19_ERROR_FRAME_INVALID,
    LD19_ERROR_TIMEOUT,
    LD19_ERROR_NULL_POINTER
} ld19_error_t;

#endif // LD19_CONFIG_H
