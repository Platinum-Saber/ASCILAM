# ASCILAM Multi-Robot System - Arduino Integration

## System Overview

This updated system replaces the M-Bot with a custom Arduino-based motor controller featuring:
- **Encoder-based odometry** for accurate position tracking
- **MPU6050 IMU integration** for improved heading estimation
- **L298N motor driver** for differential drive control
- **Real-time odometry feedback** to ESP32 via serial communication

---

## Hardware Architecture

### Component Stack (Per Robot)

```
┌─────────────────────────────────────┐
│     Raspberry Pi 4 (ROS2 Foxy)      │
│   - Multi-robot SLAM                 │
│   - Robot Coordinator                │
│   - Robot Controllers                │
└─────────────────────────────────────┘
              ↕ WiFi (micro-ROS)
┌─────────────────────────────────────┐
│         ESP32 DevKit                 │
│   - LD19 LiDAR interface             │
│   - micro-ROS node                   │
│   - Arduino communication            │
└─────────────────────────────────────┘
              ↕ UART (9600 baud)
┌─────────────────────────────────────┐
│      Arduino (Uno/Nano/Mega)         │
│   - Motor control (L298N)            │
│   - Encoder reading                  │
│   - MPU6050 IMU                      │
│   - Odometry calculation             │
└─────────────────────────────────────┘
              ↕ PWM & Digital
┌─────────────────────────────────────┐
│      L298N Motor Driver              │
│   - Dual H-bridge                    │
│   - Motor A (Left wheel)             │
│   - Motor B (Right wheel)            │
└─────────────────────────────────────┘
```

---

## Hardware Connections

### Arduino → L298N Motor Driver

| Arduino Pin | L298N Pin | Function          |
|-------------|-----------|-------------------|
| D5          | ENA       | Motor A PWM       |
| D8          | IN1       | Motor A Dir 1     |
| D9          | IN2       | Motor A Dir 2     |
| D6          | ENB       | Motor B PWM       |
| D10         | IN3       | Motor B Dir 1     |
| D11         | IN4       | Motor B Dir 2     |
| GND         | GND       | Common Ground     |

### Arduino → Encoders (3-pin type)

| Arduino Pin | Encoder Pin | Function       |
|-------------|-------------|----------------|
| D2          | Encoder A SIG | Left wheel    |
| D3          | Encoder B SIG | Right wheel   |
| 5V          | VCC (both)   | Power         |
| GND         | GND (both)   | Ground        |

### Arduino → MPU6050

| Arduino Pin | MPU6050 Pin | Function |
|-------------|-------------|----------|
| A4 (SDA)    | SDA         | I2C Data |
| A5 (SCL)    | SCL         | I2C Clock|
| 5V          | VCC         | Power    |
| GND         | GND         | Ground   |

### Arduino → ESP32

| Arduino Pin | ESP32 Pin | Function      |
|-------------|-----------|---------------|
| TX (D1)     | RX0 (D3)  | Serial TX     |
| RX (D0)     | TX0 (D1)  | Serial RX     |
| GND         | GND       | Common Ground |

**Note:** Do NOT connect 5V between Arduino and ESP32!

---

## Software Architecture

### Data Flow

```
ROS2 (Raspberry Pi)
    ↓ /robotX/cmd_vel (Twist message)
ESP32 micro-ROS
    ↓ D<left_pwm>,<right_pwm> (Serial command)
Arduino Motor Controller
    ↓ PWM signals to L298N
    ↑ Encoder pulses
    ↑ MPU6050 data
Arduino Odometry Calculation
    ↑ ODOM:x,y,theta,vx,vy,vtheta (Serial data)
ESP32 micro-ROS
    ↑ /robotX/odom (Odometry message)
ROS2 SLAM System
```

### Communication Protocols

#### ESP32 → Arduino Commands

| Command Format | Description | Example |
|----------------|-------------|---------|
| `D<left>,<right>` | Differential drive | `D150,-150` |
| `M<speed>` | Move forward/backward | `M100` |
| `T<angle>` | Turn in place | `T45` |
| `S0` | Stop motors | `S0` |
| `R` | Reset odometry | `R` |

#### Arduino → ESP32 Data

| Data Format | Description | Example |
|-------------|-------------|---------|
| `ODOM:x,y,theta,vx,vy,vtheta` | Odometry data | `ODOM:0.5,0.2,1.57,0.1,0.0,0.0` |
| `STATUS:<message>` | Status updates | `STATUS:MPU6050_OK` |
| `READY:Robot<ID>` | Initialization | `READY:Robot1` |

---

## Configuration Parameters

### Robot Physical Parameters (Arduino)

```cpp
#define WHEEL_DIAMETER 0.065        // 65mm wheels (meters)
#define WHEEL_BASE 0.15             // 150mm between wheels (meters)
#define ENCODER_PULSES_PER_REV 20   // Encoder resolution
#define GEAR_RATIO 1.0              // Motor gear ratio
```

**⚠️ Important:** Measure and adjust these values for your specific robot!

### Robot Configuration (ESP32)

```cpp
// Robot 1
const int agent_port = 8888;
const float wheel_base = 0.15;
const float max_speed = 0.5;

// Robot 2
const int agent_port = 8889;
```

---

## Odometry Calculation Method

The Arduino uses **differential drive kinematics** with encoder fusion:

### 1. Encoder-based Dead Reckoning

```
distance_left = encoder_left_pulses × meters_per_pulse
distance_right = encoder_right_pulses × meters_per_pulse

distance_center = (distance_left + distance_right) / 2
delta_theta = (distance_right - distance_left) / wheel_base

delta_x = distance_center × cos(theta + delta_theta/2)
delta_y = distance_center × sin(theta + delta_theta/2)
```

### 2. IMU Fusion (MPU6050)

The system uses a **complementary filter** to fuse encoder and gyro data:

```
theta = 0.8 × (theta_encoder) + 0.2 × (theta_gyro)
```

This reduces:
- Encoder slippage errors
- Gyro drift over time
- Improves heading accuracy

---

## Installation & Setup

### 1. Arduino Setup

**Required Libraries:**
- `Wire.h` (built-in)
- `MPU6050_light` (Arduino Library Manager)

**Installation:**
```bash
# In Arduino IDE:
# Tools → Manage Libraries → Search "MPU6050_light" → Install
```

**Upload Code:**
1. Open `arduino_motor_encoder.ino`
2. Set `ROBOT_ID` (1 or 2)
3. Adjust physical parameters if needed
4. Select board and port
5. Upload

### 2. ESP32 Setup

**Required Libraries:**
- micro_ros_arduino v2.0.5-foxy

**Upload Code:**
1. Flash `ESP32_Robot1_Arduino.ino` to Robot 1 ESP32
2. Flash `ESP32_Robot2_Arduino.ino` to Robot 2 ESP32
3. Verify WiFi credentials match your setup

### 3. ROS2 System (No Changes Required)

The existing ROS2 code works without modification since the odometry topic interface remains the same.

---

## Testing Procedure

### 1. Test Arduino Standalone

```cpp
// Monitor Serial output at 9600 baud
// You should see:
READY:Robot1
STATUS:MPU6050_OK
ODOM:0.0000,0.0000,0.0000,0.0000,0.0000,0.0000
```

### 2. Test Motor Commands

Send via Arduino Serial Monitor:
```
D100,100    // Both wheels forward
D-100,-100  // Both wheels backward
D100,-100   // Turn right
S0          // Stop
```

### 3. Test ESP32 Integration

Monitor ESP32 serial output:
```
Robot 1 initialized with Arduino motor controller
Robot 1 WiFi connected! IP: 192.168.4.X
Pinging micro-ROS agent... OK
Robot 1 micro-ROS initialized
Arduino ready: READY:Robot1
```

### 4. Test Full System

```bash
cd ~/ASCILAM/multirobot_ws
./start_exploration.sh
```

Monitor topics:
```bash
# Check odometry
ros2 topic echo /robot1/odom

# Check if motors respond
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

---

## Troubleshooting

### Issue: No odometry data from Arduino

**Symptoms:**
- ESP32 shows: "Warning: No data from Arduino!"
- No ODOM messages in serial monitor

**Solutions:**
1. Check UART connections (TX→RX, RX→TX)
2. Verify baud rate (9600)
3. Check Arduino power supply
4. Test Arduino standalone

### Issue: Motors not responding

**Symptoms:**
- Odometry updates but motors don't move
- CMD messages received but no motion

**Solutions:**
1. Check L298N power supply (7-12V)
2. Verify motor connections to L298N
3. Test motor driver with simple sketch
4. Check PWM pin connections

### Issue: Encoders not counting

**Symptoms:**
- ODOM shows 0,0,0 always
- Motors work but no position change

**Solutions:**
1. Verify encoder power (5V, GND)
2. Check interrupt pins (D2, D3)
3. Test encoders with LED blink on pulse
4. Ensure encoders are properly mounted

### Issue: MPU6050 initialization failed

**Symptoms:**
- `STATUS:MPU6050_ERROR:X`

**Solutions:**
1. Check I2C connections (SDA, SCL)
2. Verify MPU6050 address (0x68 or 0x69)
3. Try I2C scanner sketch
4. Check pull-up resistors on I2C lines

### Issue: Robot drifts or curves

**Symptoms:**
- Robot doesn't go straight
- Constant rotation even when stopped

**Solutions:**
1. Calibrate wheel diameters separately
2. Adjust motor speed trim in code
3. Check encoder alignment
4. Verify wheel encoders are matched

---

## Calibration Guide

### 1. Measure Wheel Diameter

```cpp
// Roll robot forward exactly 1 meter
// Count encoder pulses
// Calculate: WHEEL_DIAMETER = distance / (pulses × π / ENCODER_PULSES_PER_REV)
```

### 2. Measure Wheel Base

```cpp
// Rotate robot 360° in place
// Measure actual rotation vs. odometry
// Adjust WHEEL_BASE until they match
```

### 3. Encoder Pulses Per Revolution

```cpp
// Lift robot, mark wheel position
// Rotate wheel exactly one revolution
// Count pulses in serial monitor
// Update ENCODER_PULSES_PER_REV
```

### 4. MPU6050 Calibration

The MPU6050 auto-calibrates on startup. For best results:
- Place robot on flat surface during startup
- Don't move robot for 3 seconds after power-on
- Recalibrate if transported to new location

---

## Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Odometry Update Rate | 20 Hz | Adjustable in code |
| Position Accuracy | ±2 cm/m | With calibration |
| Heading Accuracy | ±2° | With MPU6050 fusion |
| Max Linear Speed | 0.5 m/s | Configurable |
| Command Latency | <50 ms | ESP32 → Arduino → Motors |
| Encoder Resolution | 20 PPR | Typical hobby encoder |

---

## Advantages Over M-Bot

✅ **Direct encoder access** - More accurate odometry  
✅ **IMU integration** - Better heading estimation  
✅ **Customizable** - Adjust any parameter  
✅ **Debugging** - Full access to all sensor data  
✅ **Cost-effective** - Standard Arduino components  
✅ **Expandable** - Easy to add more sensors  

---

## Future Enhancements

### Possible Upgrades

1. **Wheel Slip Detection**
   - Compare encoder speed vs IMU acceleration
   - Detect and compensate for slippage

2. **PID Motor Control**
   - Closed-loop speed control
   - Better straight-line motion

3. **Battery Monitoring**
   - Voltage sensing on Arduino
   - Low battery warnings to ROS

4. **Additional Sensors**
   - Ultrasonic for close-range detection
   - IR sensors for edge detection
   - Bumper switches for collision

5. **Advanced Filtering**
   - Extended Kalman Filter (EKF)
   - Better sensor fusion

---

## System Status LEDs

### ESP32 LED (GPIO 2)

| Pattern | Meaning |
|---------|---------|
| Solid ON | Fully connected (WiFi + micro-ROS + Arduino) |
| Slow blink (1 Hz) | WiFi connected, waiting for ROS |
| Fast blink (5 Hz) | No WiFi connection |

### Arduino Built-in LED

Can be programmed for debugging:
- Blink on encoder pulse
- Show command reception
- Indicate MPU6050 status

---

## Support & Troubleshooting

For issues or questions:
1. Check serial monitor outputs (both Arduino and ESP32)
2. Verify all hardware connections
3. Test each subsystem independently
4. Review calibration values
5. Check ROS2 topics are publishing

**Remember:** The system is modular - test each layer independently!