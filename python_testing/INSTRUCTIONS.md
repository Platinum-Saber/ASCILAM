# ASCILAM Python Testing Instructions

This document provides comprehensive instructions for using the RPLIDAR, LD19 LIDAR, and UDP transmission components of the ASCILAM (Autonomous Spacecraft Collision Identification and Landing Area Mapping) project.

## 📁 Project Structure

```
python_testing/
├── requirements.txt              # Python dependencies
├── INSTRUCTIONS.md              # This file
├── rplidar/                     # RPLIDAR A2 testing and visualization
│   └── lidar_test.py           # Real-time RPLIDAR data visualization
├── ldlidar/                     # LD19 LIDAR testing and ESP32 integration
│   ├── esp32_integration/       # ESP32 firmware for LD19
│   │   └── ld19lidar/          # PlatformIO project
│   │       ├── include/        # Header files
│   │       ├── src/            # Source code
│   │       └── platformio.ini  # PlatformIO configuration
│   └── functionality_test/     # LD19 testing and visualization scripts
│       ├── udp_receiver.py     # Basic UDP receiver
│       ├── udp_formatter_simple.py    # Simple frame formatter
│       ├── udp_frame_parser.py # Advanced frame parser
│       ├── udp_data_tester.py  # Data validation and testing
│       ├── udp_visualizer.py   # Real-time display
│       ├── udp_ascii_mapper.py # ASCII 2D mapping (no dependencies)
│       ├── udp_2d_mapper.py    # Advanced 2D mapping with matplotlib
│       └── udp_simple_plotter.py # Adaptive plotting
└── udp_transmission/           # Network communication modules
    ├── lysia_udp.py           # UDP receiver with real-time plotting
    ├── udp_sender.py          # LIDAR data UDP transmission
    ├── udp_test_receiver.py   # UDP testing receiver
    └── udp_test_sender.py     # UDP testing sender
```

## 🛠️ Prerequisites and Setup

### 1. Install Dependencies

First, install the required Python packages:

```bash
cd python_testing
pip install -r requirements.txt
```

**Required packages:**

- `numpy>=1.21.0` - Numerical computing
- `matplotlib>=3.5.0` - Data visualization and 2D mapping
- `rplidar>=0.9.2` - RPLIDAR communication library
- `pyserial>=3.5` - Serial communication (for RPLIDAR and ESP32)

### 2. Hardware Requirements

#### Option A: RPLIDAR A2 Setup

- **RPLIDAR A2 or compatible** - For direct LIDAR data acquisition
- **USB to Serial Adapter** - To connect LIDAR to computer

#### Option B: LD19 LIDAR + ESP32 Setup (Recommended)

- **LD19 LIDAR sensor** - Compact, affordable LIDAR sensor
- **ESP32 Development Board** - For LIDAR interface and WiFi transmission
- **WiFi Network** - For wireless data transmission
- **Jumper wires and breadboard** - For connections

#### General Requirements

- **Network connection** - For UDP transmission between devices
- **Computer/Raspberry Pi** - For data reception and visualization

### 3. Hardware Setup

#### RPLIDAR A2 Setup

1. Connect the RPLIDAR to your computer via USB
2. Open Device Manager (Windows) or check `/dev/tty*` (Linux)
3. Note the COM port (e.g., `COM2`, `COM9`)
4. Update the `PORT_NAME` variable in the relevant scripts

#### LD19 + ESP32 Setup

**Wiring:**

- ESP32 GPIO16 (RX2) ← LD19 TX (Yellow wire)
- ESP32 GPIO17 (TX2) → Unused (but must be assigned)
- ESP32 GND ← LD19 GND (Black wire)
- ESP32 5V/VIN ← LD19 VCC (Red wire)

**Configuration:**

1. Install PlatformIO IDE or Arduino IDE
2. Open the project in `ldlidar/esp32_integration/ld19lidar/`
3. Configure WiFi credentials in `main.cpp`
4. Choose operation mode (raw hex, parsed frames, UDP forwarding)
5. Upload to ESP32

## 🔍 RPLIDAR Module (`rplidar/`)

### `lidar_test.py` - Real-Time LIDAR Visualization

**Purpose:** Connects to RPLIDAR and displays real-time scan data in a polar plot.

**Features:**

- Real-time polar plot visualization
- 1-meter radius filtering (600mm limit)
- Device health monitoring
- Interrupt-safe operation

**Configuration:**

```python
PORT_NAME = 'COM2'  # Change to your actual COM port
```

**Usage:**

```bash
cd rplidar
python lidar_test.py
```

**Controls:**

- Press `Ctrl+C` to stop scanning safely
- The plot updates in real-time showing objects within 600mm

**Output:**

- Polar plot with North at top
- Blue dots represent detected objects
- Distance scale: 0-600mm
- Real-time updates

## 🎯 LD19 LIDAR Module (`ldlidar/`)

### ESP32 Integration (`esp32_integration/ld19lidar/`)

**Purpose:** Complete ESP32 firmware for interfacing with LD19 LIDAR and transmitting data via WiFi.

**Features:**

- Modular, object-oriented C++ code
- Multiple operation modes
- Real-time frame parsing
- WiFi UDP transmission
- Independent function usage

#### Project Structure

```
ld19lidar/
├── platformio.ini          # PlatformIO configuration
├── include/               # Header files
│   ├── ld19_config.h     # Configuration constants
│   ├── ld19_reader.h     # Raw serial reading
│   ├── ld19_parser.h     # Frame parsing
│   └── ld19_udp.h        # UDP transmission
├── src/                  # Source files
│   ├── main.cpp          # Main application
│   ├── examples.cpp      # Usage examples
│   ├── read_serial.cpp   # Reader implementation
│   ├── format_serial_data.cpp  # Parser implementation
│   └── udp_data_sender.cpp     # UDP implementation
└── README.md            # Detailed documentation
```

#### Operation Modes

Edit `main.cpp` and uncomment ONE mode:

```cpp
#define MODE_RAW_HEX        // Raw hex dump for debugging
#define MODE_PARSED_FRAMES  // Structured data display
#define MODE_UDP_FORWARD    // Network transmission
#define MODE_COMBO_PARSE_UDP // Combined mode
```

#### Configuration

Update WiFi settings in `main.cpp`:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* PC_IP         = "192.168.1.50";   // Target computer IP
const uint16_t UDP_PORT   = 5005;             // UDP port
```

#### Usage

1. **Build and Upload:**

   ```bash
   cd ldlidar/esp32_integration/ld19lidar
   pio run --target upload
   ```

2. **Monitor Serial Output:**

   ```bash
   pio device monitor
   ```

3. **Independent Function Usage:**

   ```cpp
   // Use only raw reading
   LD19Reader reader;
   reader.begin();
   reader.printRawHex();

   // Use only parsing
   LD19Parser parser(serial_instance);
   LD19Frame frame;
   if (parser.readFrame(frame)) {
       // Process structured data
   }

   // Use only UDP transmission
   LD19UDP udp("SSID", "PASS", "IP", 5005);
   udp.sendRawFrame(frame_data, 47);
   ```

### Functionality Testing (`functionality_test/`)

**Purpose:** Comprehensive Python scripts for receiving, parsing, and visualizing LD19 LIDAR data transmitted via UDP.

#### Core Scripts

##### `udp_formatter_simple.py` - Recommended Starting Point ⭐

**Purpose:** Clear, detailed formatting of received UDP frames.

**Features:**

- Frame validation and error detection
- Detailed breakdown of all 12 measurement points
- Visual bar charts for close objects
- Comprehensive frame statistics

**Usage:**

```bash
cd ldlidar/functionality_test
python udp_formatter_simple.py
```

**Output Example:**

```
================================================================================
Frame received at [14:23:15.123] from 192.168.1.100:5005
Data length: 47 bytes (expected: 47)
Header: 0x54 ✓
VerLen: 0x2C ✓
Speed: 450 degrees/second
Start Angle: 125.67° (raw: 12567)
End Angle: 155.23° (raw: 15523)

Measurement Points (12 points):
Point Angle    Distance   Intensity Status
----- -------- ---------- --------- ----------
 0    125.67°    856 mm      127     Valid
 1    128.35°    923 mm      145     Valid
...

Close Objects (< 1 meter):
  Point  0 at 125.7°: 856mm |████████████████████████████████|
```

##### `udp_ascii_mapper.py` - 2D Mapping (No Dependencies) 🗺️

**Purpose:** Real-time 2D ASCII mapping without external dependencies.

**Features:**

- Text-based 2D map visualization
- Real-time accumulation of scan data
- Distance-based symbols (█▓▒░·)
- Works on any system without matplotlib

**Usage:**

```bash
python udp_ascii_mapper.py
```

**Output:**

```
🎯 LD19 LIDAR ASCII Map Visualizer
================================================================================
Frame #47 | Speed: 450 dps | Range: 125.7°-155.2° | Points: 8
Valid points: 234/276 | Map: 8x8m @ 10.0cm/char
================================================================================
                              ░    ▒▒▓█████▓▒░
                         ░▒▓██████████████████████▓▒░
                    ░▒▓████████████████L████████████▓▒░
                 ░▒▓███████████████████████████████████▓▒░
================================================================================
Legend: L=LIDAR █=Very close ▓=Close ▒=Medium ░=Far ·=Very far
```

##### `udp_2d_mapper.py` - Advanced Graphical Mapping 📊

**Purpose:** High-quality 2D mapping with matplotlib visualization.

**Requirements:** `matplotlib`, `numpy`

**Features:**

- Dual-plot display (accumulated map + current scan)
- Color-coded distance visualization
- Interactive controls (c=clear, s=save, q=quit)
- Export to PNG files

**Usage:**

```bash
pip install matplotlib numpy
python udp_2d_mapper.py
```

##### `udp_visualizer.py` - Real-time Display 🎯

**Purpose:** Live updating terminal display with polar visualization.

**Features:**

- Full-screen real-time updates
- Polar coordinate display with emojis
- Close object alerts
- Frame statistics

**Usage:**

```bash
python udp_visualizer.py
```

##### `udp_data_tester.py` - Validation and Testing 🔍

**Purpose:** Comprehensive data validation and quality analysis.

**Features:**

- Frame validation and error analysis
- Statistics tracking and reporting
- JSON export capabilities
- Interactive testing commands

**Usage:**

```bash
python udp_data_tester.py
```

**Interactive Commands:**

- `s` + Enter: Show statistics
- `d` + Enter: Toggle detailed output
- `q` + Enter: Quit
- `save` + Enter: Save results to JSON

#### Quick Start Guide

1. **Setup ESP32:**

   - Wire LD19 to ESP32 as shown above
   - Configure WiFi in `main.cpp`
   - Upload firmware with UDP forwarding mode

2. **Start with Simple Formatting:**

   ```bash
   python udp_formatter_simple.py
   ```

3. **Try ASCII Mapping:**

   ```bash
   python udp_ascii_mapper.py
   ```

4. **Advanced Visualization (if matplotlib available):**
   ```bash
   python udp_2d_mapper.py
   ```

#### Frame Format

LD19 frames are 47 bytes with this structure:

| Bytes | Content     | Description                                |
| ----- | ----------- | ------------------------------------------ |
| 0     | Header      | Always 0x54                                |
| 1     | VerLen      | Always 0x2C (12 points)                    |
| 2-3   | Speed       | Rotation speed in degrees/second           |
| 4-5   | Start Angle | Start angle in hundredths of degrees       |
| 6-41  | Points      | 12 points × 3 bytes (distance + intensity) |
| 42-43 | End Angle   | End angle in hundredths of degrees         |
| 44-45 | Timestamp   | Timestamp in milliseconds                  |
| 46    | CRC8        | Checksum                                   |

## 🌐 UDP Transmission Module (`udp_transmission/`)

### Network Architecture

The UDP transmission system enables real-time LIDAR data sharing across network devices:

```
[LIDAR Device] --> [udp_sender.py] --> UDP Network --> [lysia_udp.py/Receiver]
```

### `udp_sender.py` - LIDAR Data Transmitter

**Purpose:** Captures LIDAR data and streams it via UDP to a remote device.

**Features:**

- Real-time LIDAR data acquisition
- 1-meter distance filtering
- JSON data serialization
- Network error handling

**Configuration:**

```python
LIDAR_PORT = 'COM9'              # Your LIDAR COM port
UDP_IP = '192.168.69.60'         # Target device IP (Raspberry Pi)
UDP_PORT = 5005                  # Target UDP port
```

**Usage:**

```bash
cd udp_transmission
python udp_sender.py
```

**Data Format:**

```json
[[angle1, distance1], [angle2, distance2], ...]
```

Example: `[[0, 500], [90, 750], [180, 300]]`

### `lysia_udp.py` - UDP Receiver with Visualization

**Purpose:** Receives UDP LIDAR data and displays it in real-time polar plots.

**Features:**

- UDP socket listening
- Real-time polar plot updates
- Network timeout handling
- Debug output for troubleshooting

**Configuration:**

```python
UDP_IP = '0.0.0.0'  # Listen on all interfaces
UDP_PORT = 5005     # Listening port
```

**Usage:**

```bash
cd udp_transmission
python lysia_udp.py
```

**Output:**

- Green dots in polar plot
- Console debug messages
- Network status information

### Testing Components

#### `udp_test_sender.py` - Network Test Transmitter

**Purpose:** Sends test data to verify UDP communication.

**Features:**

- Sends predefined test data
- No LIDAR hardware required
- Quick network verification

**Configuration:**

```python
UDP_IP = "192.168.69.60"  # Target IP
UDP_PORT = 5005           # Target port
```

**Usage:**

```bash
cd udp_transmission
python udp_test_sender.py
```

#### `udp_test_receiver.py` - Network Test Receiver

**Purpose:** Comprehensive UDP data reception testing and debugging.

**Features:**

- Detailed message analysis
- JSON parsing validation
- Connection diagnostics
- Message counting

**Usage:**

```bash
cd udp_transmission
python udp_test_receiver.py
```

## 🚀 Usage Workflows

### Workflow 1: Local RPLIDAR Testing

1. Connect RPLIDAR to computer
2. Identify COM port
3. Update `PORT_NAME` in `lidar_test.py`
4. Run: `python rplidar/lidar_test.py`

### Workflow 2: LD19 + ESP32 Setup (Recommended)

**Step 1: Hardware Setup**

1. Wire LD19 to ESP32 (see wiring diagram above)
2. Connect ESP32 to computer via USB
3. Power on both devices

**Step 2: ESP32 Firmware**

1. Open `ldlidar/esp32_integration/ld19lidar/` in PlatformIO
2. Configure WiFi credentials in `main.cpp`
3. Choose operation mode (recommend `MODE_UDP_FORWARD`)
4. Build and upload: `pio run --target upload`

**Step 3: Data Reception**

1. On receiving computer, run one of:

   ```bash
   # Simple formatting (recommended first)
   python ldlidar/functionality_test/udp_formatter_simple.py

   # ASCII mapping (no dependencies)
   python ldlidar/functionality_test/udp_ascii_mapper.py

   # Advanced mapping (requires matplotlib)
   python ldlidar/functionality_test/udp_2d_mapper.py
   ```

### Workflow 3: Network RPLIDAR Streaming

**On LIDAR Device (Data Source):**

1. Configure `udp_sender.py` with correct COM port and target IP
2. Run: `python udp_transmission/udp_sender.py`

**On Receiving Device (e.g., Raspberry Pi):**

1. Run: `python udp_transmission/lysia_udp.py`

### Workflow 4: Network Testing and Validation

**Testing UDP Connection:**

1. On receiver: `python udp_transmission/udp_test_receiver.py`
2. On sender: `python udp_transmission/udp_test_sender.py`
3. Verify data reception and parsing

**LD19 Data Validation:**

1. Run ESP32 in UDP mode
2. On computer: `python ldlidar/functionality_test/udp_data_tester.py`
3. Use interactive commands to analyze data quality

### Workflow 5: 2D Mapping and Visualization

**Real-time ASCII Mapping (No Dependencies):**

```bash
python ldlidar/functionality_test/udp_ascii_mapper.py
```

**Advanced Graphical Mapping:**

```bash
pip install matplotlib numpy
python ldlidar/functionality_test/udp_2d_mapper.py
```

**Controls:**

- ASCII Mapper: Automatic updates, Ctrl+C to exit
- Graphical Mapper: 'c'=clear map, 's'=save PNG, 'q'=quit

## ⚙️ Configuration Guide

### Network Configuration

1. **IP Address Setup:**

   - Ensure devices are on the same network
   - Update `UDP_IP` variables with correct target IPs
   - Use `192.168.x.x` for local networks
   - For LD19+ESP32: Configure in `main.cpp`

2. **Port Configuration:**

   - Default port: `5005`
   - Ensure ports are not blocked by firewalls
   - Use different ports if conflicts occur

3. **Firewall Settings:**
   - Allow Python applications through firewall
   - Open UDP port 5005 (or your chosen port)
   - For ESP32: Ensure WiFi network allows device communication

### LIDAR Configuration

#### RPLIDAR A2 Configuration

1. **COM Port Detection:**

   ```bash
   # Windows
   # Check Device Manager > Ports (COM & LPT)

   # Linux
   ls /dev/tty* | grep USB
   ```

2. **Baud Rate:** 115200 (default for RPLIDAR A2)

3. **Range Settings:**
   - Current limit: 1000mm (1 meter)
   - Modify filtering in code as needed

#### LD19 + ESP32 Configuration

1. **ESP32 Programming:**

   - Install PlatformIO IDE or Arduino IDE with ESP32 support
   - Install required libraries (built-in WiFi, HardwareSerial)

2. **LD19 Settings:**

   - Baud Rate: 230400 (fixed for LD19)
   - Frame Rate: ~10-13 Hz
   - Range: 0.02m - 12m
   - Resolution: ~0.33° per point

3. **WiFi Configuration:**

   ```cpp
   const char* WIFI_SSID     = "YOUR_NETWORK_NAME";
   const char* WIFI_PASSWORD = "YOUR_NETWORK_PASSWORD";
   const char* PC_IP         = "192.168.1.100";  // Target computer
   const uint16_t UDP_PORT   = 5005;
   ```

4. **Operation Mode Selection:**
   - `MODE_RAW_HEX`: Debug mode, shows raw bytes
   - `MODE_PARSED_FRAMES`: Structured data display
   - `MODE_UDP_FORWARD`: Network transmission (recommended)
   - `MODE_COMBO_PARSE_UDP`: Combined local display + network

## 🐛 Troubleshooting

### Common Issues

**RPLIDAR Connection Problems:**

- Verify COM port is correct
- Check cable connections
- Ensure LIDAR is powered
- Try different USB ports

**LD19 + ESP32 Issues:**

- **No serial data:** Check wiring, ensure LD19 is powered (5V)
- **WiFi connection fails:** Verify SSID/password, check signal strength
- **No UDP data received:** Check IP addresses, firewall settings
- **Frame sync errors:** Verify baud rate (230400), check connections
- **ESP32 won't program:** Hold BOOT button while uploading

**UDP Communication Issues:**

- Verify IP addresses are correct
- Check network connectivity: `ping [target_ip]`
- Ensure firewall allows UDP traffic
- Confirm port availability
- For ESP32: Check ESP32 IP with `Serial.println(WiFi.localIP())`

**Python Script Issues:**

- **Import Errors:** Install requirements: `pip install -r requirements.txt`
- **Matplotlib not found:** Install with `pip install matplotlib numpy`
- **Permission denied:** Run with administrator privileges or use port > 1024
- **No data displayed:** Check UDP IP and port configuration

**Performance Issues:**

- Reduce plot update frequency
- Filter data more aggressively
- Close unnecessary applications
- For ESP32: Add small delays in main loop if needed

### Debug Commands

```bash
# Test network connectivity
ping 192.168.1.100

# Check port usage
netstat -an | findstr "5005"    # Windows
netstat -an | grep "5005"       # Linux

# List COM ports (Windows)
mode

# Python package verification
pip list | grep -E "(rplidar|matplotlib|numpy)"

# ESP32 specific debugging
pio device monitor              # Monitor ESP32 serial output
pio device list                # List connected devices
```

### ESP32 Specific Debugging

1. **Monitor Serial Output:**

   ```bash
   pio device monitor
   ```

2. **Check WiFi Connection:**

   - Look for "WiFi OK. IP: xxx.xxx.xxx.xxx" message
   - Note the ESP32's IP address

3. **Validate LIDAR Data:**

   - In `MODE_RAW_HEX`: Should see continuous hex data
   - In `MODE_PARSED_FRAMES`: Should see structured frame info

4. **Network Testing:**
   - Use `udp_data_tester.py` for comprehensive validation
   - Check frame validity rates (should be >90%)

### Data Quality Indicators

**Good LD19 Performance:**

- Frame validity rate > 90%
- Consistent speed readings (400-600 dps typical)
- Distance measurements in expected range (20mm - 12m)
- Regular frame reception (10-13 Hz)

**Signs of Problems:**

- Low frame validity rate (< 80%)
- Erratic speed readings or zero values
- Many "No echo" measurements indoors
- Irregular frame timing or dropouts

## 📊 Data Format Specifications

### RPLIDAR Data Structure

```python
# Raw LIDAR scan format
scan = [
    (quality, angle, distance),
    (quality, angle, distance),
    ...
]

# Transmitted UDP format (filtered)
transmitted_data = [
    [angle, distance],
    [angle, distance],
    ...
]
```

### LD19 LIDAR Data Structure

#### Raw Frame Format (47 bytes)

```
Byte 0:    Header (0x54)
Byte 1:    VerLen (0x2C = 12 points)
Bytes 2-3: Speed (degrees per second, little-endian)
Bytes 4-5: Start angle (hundredths of degree, little-endian)
Bytes 6-41: 12 measurement points (3 bytes each)
  - Bytes 0-1: Distance in mm (little-endian)
  - Byte 2:    Intensity (0-255)
Bytes 42-43: End angle (hundredths of degree, little-endian)
Bytes 44-45: Timestamp in ms (little-endian)
Byte 46:     CRC8 checksum
```

#### Parsed Frame Structure

```python
# Python parsed frame
frame = {
    'speed_dps': 450,
    'start_angle': 125.67,    # degrees
    'end_angle': 155.23,      # degrees
    'timestamp_ms': 1234,
    'points': [
        {'distance_mm': 856, 'intensity': 127},
        {'distance_mm': 923, 'intensity': 145},
        # ... 12 points total
    ]
}
```

#### ESP32 C++ Frame Structure

```cpp
struct LD19Point {
    uint16_t distance_mm;
    uint8_t intensity;
};

struct LD19Frame {
    uint16_t speed_dps;
    uint16_t start_angle;     // hundredths of degree
    uint16_t end_angle;       // hundredths of degree
    uint16_t timestamp_ms;
    uint8_t crc8;
    LD19Point points[12];
    bool valid;
};
```

### JSON Message Format

#### RPLIDAR UDP Messages

```json
{
  "type": "lidar_scan",
  "timestamp": "2025-08-04T10:30:00Z",
  "data": [
    [0.0, 450.5],
    [1.5, 432.1],
    [3.0, 501.7]
  ]
}
```

#### LD19 UDP Messages (Raw Binary)

- Direct transmission of 47-byte frames
- No JSON encoding for maximum efficiency
- Received as binary data for parsing

## 📝 Development Notes

### System Architecture

**RPLIDAR A2 System:**

```
[RPLIDAR A2] --USB--> [Computer] --UDP--> [Remote Display]
```

**LD19 + ESP32 System (Recommended):**

```
[LD19] --UART--> [ESP32] --WiFi/UDP--> [Computer/Raspberry Pi]
```

### Performance Characteristics

**RPLIDAR A2:**

- Range: 0.15m - 8m
- Frequency: 5.5 Hz
- Resolution: ~0.9° (400 points per scan)
- Interface: USB Serial (115200 baud)

**LD19 LIDAR:**

- Range: 0.02m - 12m
- Frequency: 10-13 Hz
- Resolution: ~0.33° (12 points per frame, 30° range typical)
- Interface: UART (230400 baud)

### Key Features

- **Real-time Performance:** Scripts optimized for real-time data processing
- **Error Handling:** Robust error handling for network and hardware failures
- **Modularity:** Each component can be used independently
- **Extensibility:** Easy to modify for different LIDAR models or network protocols
- **Cross-platform:** Works on Windows, Linux, and Raspberry Pi
- **No Dependencies Option:** ASCII mapping works without external libraries

### ESP32 Code Architecture

- **Object-Oriented Design:** Separate classes for reading, parsing, and UDP transmission
- **Independent Functions:** Each class can be used standalone
- **Memory Efficient:** Optimized for ESP32's limited RAM
- **Real-time Capable:** Non-blocking operations where possible
- **Configurable:** Easy mode switching and parameter adjustment

### Python Script Categories

1. **Data Reception:** Basic UDP receiving and validation
2. **Data Formatting:** Human-readable display of frame contents
3. **Real-time Visualization:** Live updating displays and plots
4. **2D Mapping:** Spatial visualization and mapping
5. **Testing & Validation:** Quality analysis and debugging tools

### Recommended Development Workflow

1. **Start with ESP32 raw mode** to verify LIDAR connectivity
2. **Use simple UDP formatter** to validate data transmission
3. **Progress to ASCII mapping** for spatial understanding
4. **Add matplotlib visualization** for detailed analysis
5. **Implement custom processing** based on application needs

## 🔒 Safety Considerations

- **RPLIDAR:** Always use `Ctrl+C` to stop LIDAR operations safely
- **LD19 + ESP32:** Ensure proper power supply (5V for LD19, 3.3V/5V for ESP32)
- **Electrical Safety:** Double-check wiring before powering on
- **Network Security:** Use secure WiFi networks for ESP32 communication
- **Data Privacy:** Be aware of environmental mapping data sensitivity
- **Hardware Protection:** Ensure proper LIDAR shutdown to prevent hardware damage
- **Network Usage:** Monitor network bandwidth when streaming data
- **Production Use:** Implement appropriate error handling and logging

## 📈 Performance Optimization Tips

### ESP32 Optimization

- Use appropriate delays to prevent overwhelming the network
- Monitor heap memory usage for long-running applications
- Implement watchdog timers for production deployments
- Use appropriate task priorities if using FreeRTOS features

### Python Script Optimization

- Limit plot update frequencies for better performance
- Use efficient data structures for large datasets
- Implement data filtering to reduce processing load
- Consider multiprocessing for CPU-intensive operations

### Network Optimization

- Use appropriate buffer sizes for UDP sockets
- Implement packet loss detection and recovery
- Consider data compression for bandwidth-limited networks
- Monitor network latency and adjust timeouts accordingly

---

**Project:** ASCILAM (Autonomous Spacecraft Collision Identification and Landing Area Mapping)  
**Last Updated:** August 2025  
**Maintainer:** CS3283 Embedded Systems Project Team

## 🎯 Quick Reference

### Essential Commands

```bash
# ESP32 Development
cd ldlidar/esp32_integration/ld19lidar
pio run --target upload
pio device monitor

# Basic UDP Reception
python ldlidar/functionality_test/udp_formatter_simple.py

# ASCII Mapping (No Dependencies)
python ldlidar/functionality_test/udp_ascii_mapper.py

# Advanced Mapping (Requires matplotlib)
pip install matplotlib numpy
python ldlidar/functionality_test/udp_2d_mapper.py

# Data Validation
python ldlidar/functionality_test/udp_data_tester.py

# RPLIDAR Testing
python rplidar/lidar_test.py
```

### Default Network Settings

- **UDP Port:** 5005
- **ESP32 → Computer:** Configured in ESP32 `main.cpp`
- **Default IP:** 192.168.1.50 (update as needed)

### Hardware Pin Assignments

- **LD19 → ESP32:** TX(Yellow)→GPIO16, GND(Black)→GND, VCC(Red)→5V
- **ESP32 Programming:** USB cable to computer
