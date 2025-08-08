# LD19 LIDAR ESP32 Integration

This project provides a modular, object-oriented approach to interfacing with the LD19 LIDAR sensor using an ESP32 microcontroller.

## Project Structure

```
include/
├── ld19_config.h      # Configuration constants and definitions
├── ld19_reader.h      # Raw serial data reading functionality
├── ld19_parser.h      # Frame parsing and formatting
└── ld19_udp.h         # UDP transmission capabilities

src/
├── main.cpp           # Main application with multiple operation modes
├── read_serial.cpp    # LD19Reader class implementation
├── format_serial_data.cpp # LD19Parser class implementation
└── udp_data_sender.cpp    # LD19UDP class implementation
```

## Features

### LD19Reader Class

- Raw serial data reading from LD19 LIDAR
- Hardware serial configuration (UART2, 230400 baud)
- Hex dump functionality for debugging
- Independent byte-by-byte reading

### LD19Parser Class

- Frame synchronization and parsing
- Structured data extraction (speed, angles, points)
- Frame validation and error detection
- Both raw and structured data access
- Formatted output for debugging

### LD19UDP Class

- WiFi connection management
- UDP packet transmission
- Real-time frame forwarding
- Connection status monitoring
- Error handling and recovery

## Hardware Setup

- **ESP32 GPIO16 (RX2)** ← LD19 TX (Yellow wire)
- **ESP32 GPIO17 (TX2)** → Unused (but must be assigned)
- **ESP32 GND** ← LD19 GND (Black wire)
- **ESP32 5V/VIN** ← LD19 VCC (Red wire)

## Usage Examples

### 1. Basic Raw Data Reading

```cpp
#include "ld19_reader.h"

LD19Reader reader;

void setup() {
    Serial.begin(115200);
    reader.begin(230400);
}

void loop() {
    reader.printRawHex();
}
```

### 2. Frame Parsing

```cpp
#include "ld19_reader.h"
#include "ld19_parser.h"

LD19Reader reader;
LD19Parser parser;

void setup() {
    Serial.begin(115200);
    reader.begin(230400);
    parser.setSerial(reader.getSerial());
}

void loop() {
    LD19Frame frame;
    if (parser.readFrame(frame)) {
        parser.printFrame(frame);

        // Access individual points
        for (int i = 0; i < 12; i++) {
            uint16_t distance = frame.points[i].distance_mm;
            uint8_t intensity = frame.points[i].intensity;
            // Process point data...
        }
    }
}
```

### 3. UDP Transmission

```cpp
#include "ld19_reader.h"
#include "ld19_udp.h"

LD19Reader reader;
LD19UDP udp_sender;

void setup() {
    Serial.begin(115200);
    reader.begin(230400);

    udp_sender.setSerial(reader.getSerial());
    udp_sender.connectWiFi("YOUR_SSID", "YOUR_PASSWORD");
    udp_sender.setTarget("192.168.1.100", 5005);
    udp_sender.setupUDP();
}

void loop() {
    udp_sender.forwardingLoop();
}
```

### 4. Combined Operations

```cpp
#include "ld19_reader.h"
#include "ld19_parser.h"
#include "ld19_udp.h"

LD19Reader reader;
LD19Parser parser;
LD19UDP udp_sender;

void setup() {
    // Initialize all components
    reader.begin(230400);
    parser.setSerial(reader.getSerial());
    udp_sender.setSerial(reader.getSerial());
    udp_sender.connectWiFi("YOUR_SSID", "YOUR_PASSWORD");
    udp_sender.setupUDP();
}

void loop() {
    // Get raw frame
    uint8_t raw_frame[47];
    if (parser.getRawFrame(raw_frame)) {
        // Send via UDP
        udp_sender.sendRawFrame(raw_frame, 47);

        // Also parse for local processing
        LD19Frame parsed_frame;
        if (parser.readFrame(parsed_frame)) {
            // Process parsed data...
        }
    }
}
```

## Configuration

### WiFi Settings

Edit the following in `main.cpp` or your application:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* PC_IP         = "192.168.1.50";
const uint16_t UDP_PORT   = 5005;
```

### Operation Modes

In `main.cpp`, uncomment one of these modes:

```cpp
#define MODE_RAW_HEX        // Raw hex dump
#define MODE_PARSED_FRAMES  // Parsed frame display
#define MODE_UDP_FORWARD    // UDP forwarding
#define MODE_COMBO_PARSE_UDP // Combined mode
```

## Frame Structure

LD19 frames are 47 bytes with the following structure:

- **Byte 0**: Header (0x54)
- **Byte 1**: VerLen (0x2C)
- **Bytes 2-3**: Speed (degrees per second)
- **Bytes 4-5**: Start angle (hundredths of degree)
- **Bytes 6-41**: 12 measurement points (3 bytes each: 2 bytes distance + 1 byte intensity)
- **Bytes 42-43**: End angle (hundredths of degree)
- **Bytes 44-45**: Timestamp (milliseconds)
- **Byte 46**: CRC8 checksum

## Error Handling

All classes include comprehensive error handling:

- Connection timeouts
- Frame validation
- WiFi disconnection recovery
- Invalid data rejection
- Serial communication errors

## Independent Function Usage

Each class is designed to work independently:

```cpp
// Use just the reader
LD19Reader standalone_reader;
standalone_reader.begin();

// Use just the parser (with external serial)
HardwareSerial mySerial(2);
mySerial.begin(230400, SERIAL_8N1, 16, 17);
LD19Parser standalone_parser(&mySerial);

// Use just UDP (with external data)
LD19UDP standalone_udp("SSID", "PASS", "IP", 5005);
uint8_t my_frame[47] = {...};
standalone_udp.sendRawFrame(my_frame, 47);
```

## Building and Flashing

1. Open in PlatformIO or Arduino IDE
2. Select "ESP32 Dev Module" as board
3. Set upload speed to 921600 baud
4. Configure your WiFi credentials in main.cpp
5. Choose operation mode by uncommenting appropriate #define
6. Build and upload

## Troubleshooting

- **No data received**: Check wiring and power supply
- **WiFi connection fails**: Verify SSID/password and signal strength
- **Frame sync issues**: Check baud rate (should be 230400)
- **UDP transmission fails**: Verify target IP and port, check network connectivity

## License

This project is provided as-is for educational and development purposes.
