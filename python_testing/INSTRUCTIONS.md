# ASCILAM Python Testing Instructions

This document provides comprehensive instructions for using the RPLIDAR and UDP transmission components of the ASCILAM (Autonomous Spacecraft Collision Identification and Landing Area Mapping) project.

## 📁 Project Structure

```
python_testing/
├── requirements.txt           # Python dependencies
├── rplidar/                  # LIDAR testing and visualization
│   └── lidar_test.py        # Real-time LIDAR data visualization
└── udp_transmission/        # Network communication modules
    ├── lysia_udp.py        # UDP receiver with real-time plotting
    ├── udp_sender.py       # LIDAR data UDP transmission
    ├── udp_test_receiver.py # UDP testing receiver
    └── udp_test_sender.py  # UDP testing sender
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
- `matplotlib>=3.5.0` - Data visualization
- `rplidar=0.9.2` - RPLIDAR communication library
- `pyserial>=3.5` - Serial communication (dependency for rplidar)

### 2. Hardware Requirements

- **RPLIDAR A2 or compatible** - For LIDAR data acquisition
- **USB to Serial Adapter** - To connect LIDAR to computer
- **Network connection** - For UDP transmission between devices
- **Raspberry Pi** (optional) - For remote data reception

### 3. COM Port Configuration

Before running LIDAR scripts, identify your LIDAR's COM port:

1. Connect the RPLIDAR to your computer
2. Open Device Manager (Windows) or check `/dev/tty*` (Linux)
3. Note the COM port (e.g., `COM2`, `COM9`)
4. Update the `PORT_NAME` variable in the relevant scripts

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

### Workflow 1: Local LIDAR Testing

1. Connect RPLIDAR to computer
2. Identify COM port
3. Update `PORT_NAME` in `lidar_test.py`
4. Run: `python rplidar/lidar_test.py`

### Workflow 2: Network LIDAR Streaming

**On LIDAR Device (Data Source):**

1. Configure `udp_sender.py` with correct COM port and target IP
2. Run: `python udp_transmission/udp_sender.py`

**On Receiving Device (e.g., Raspberry Pi):**

1. Run: `python udp_transmission/lysia_udp.py`

### Workflow 3: Network Testing

**Testing UDP Connection:**

1. On receiver: `python udp_transmission/udp_test_receiver.py`
2. On sender: `python udp_transmission/udp_test_sender.py`
3. Verify data reception and parsing

## ⚙️ Configuration Guide

### Network Configuration

1. **IP Address Setup:**

   - Ensure devices are on the same network
   - Update `UDP_IP` variables with correct target IPs
   - Use `192.168.x.x` for local networks

2. **Port Configuration:**

   - Default port: `5005`
   - Ensure ports are not blocked by firewalls
   - Use different ports if conflicts occur

3. **Firewall Settings:**
   - Allow Python applications through firewall
   - Open UDP port 5005 (or your chosen port)

### LIDAR Configuration

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

## 🐛 Troubleshooting

### Common Issues

**LIDAR Connection Problems:**

- Verify COM port is correct
- Check cable connections
- Ensure LIDAR is powered
- Try different USB ports

**UDP Communication Issues:**

- Verify IP addresses are correct
- Check network connectivity: `ping [target_ip]`
- Ensure firewall allows UDP traffic
- Confirm port availability

**Import Errors:**

- Install requirements: `pip install -r requirements.txt`
- Check Python version compatibility
- Verify virtual environment activation

**Performance Issues:**

- Reduce plot update frequency
- Filter data more aggressively
- Close unnecessary applications

### Debug Commands

```bash
# Test network connectivity
ping 192.168.69.60

# Check port usage
netstat -an | find "5005"

# List COM ports (Windows)
mode

# Python package verification
pip list | grep rplidar
```

## 📊 Data Format Specifications

### LIDAR Data Structure

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

### JSON Message Format

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

## 📝 Development Notes

- **Real-time Performance:** Scripts optimized for real-time data processing
- **Error Handling:** Robust error handling for network and hardware failures
- **Modularity:** Each component can be used independently
- **Extensibility:** Easy to modify for different LIDAR models or network protocols

## 🔒 Safety Considerations

- Always use `Ctrl+C` to stop LIDAR operations safely
- Ensure proper LIDAR shutdown to prevent hardware damage
- Monitor network bandwidth when streaming data
- Use appropriate error handling in production environments

---

**Project:** ASCILAM (Autonomous Spacecraft Collision Identification and Landing Area Mapping)  
**Created:** August 2025  
**Maintainer:** CS3283 Embedded Systems Project Team
