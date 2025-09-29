# ASCILAM Multi-Robot System: ROS 2 System Startup and Component Testing Guide

## Prerequisites

Ensure you have completed the setup using the configuration scripts:

1. `ros2_foxy_setup.sh` - ROS 2 Foxy installation
2. `setup_ros2_env.sh` - Workspace and micro-ROS setup
3. `setup_wifi_ap.sh` - WiFi Access Point configuration

---

## Starting the ROS 2 System

### 1. Navigate to Workspace

```bash
cd ~/ASCILAM/multirobot_ws
```

### 2. Source the Environment

```bash
source install/local_setup.bash
```

### 3. Launch the Multi-Robot System

```bash
ros2 launch multirobot_nav multi_robot_launch.py
```

### 4. Expected Startup Output

You should see all components starting successfully:

```
[INFO] [micro_ros_agent-1]: process started with pid [XXXX]
[INFO] [micro_ros_agent-2]: process started with pid [XXXX]
[INFO] [robot_coordinator-3]: process started with pid [XXXX]
[INFO] [robot_controller-4]: process started with pid [XXXX]
[INFO] [robot_controller-5]: process started with pid [XXXX]
[INFO] [multi_robot_slam-6]: process started with pid [XXXX]
[INFO] [static_transform_publisher-7]: process started with pid [XXXX]
[INFO] [static_transform_publisher-8]: process started with pid [XXXX]
[INFO] [static_transform_publisher-9]: process started with pid [XXXX]
[INFO] [static_transform_publisher-10]: process started with pid [XXXX]
```

Followed by initialization messages:

```
[robot_coordinator-3] [INFO] [...] [robot_coordinator]: Robot Coordinator initialized
[multi_robot_slam-6] [INFO] [...] [multi_robot_slam]: Multi-Robot SLAM initialized
[robot_controller-4] [INFO] [...] [robot1_controller]: Robot Controller initialized for robot1
[robot_controller-5] [INFO] [...] [robot2_controller]: Robot Controller initialized for robot2
```

---

## Component Testing and Verification

### 1. Check Running Nodes

Open a new terminal and run:

```bash
ros2 node list
```

Expected output:

```
/map_to_robot1_odom
/map_to_robot2_odom
/multi_robot_slam
/robot1_base_to_lidar
/robot1_controller
/robot2_base_to_lidar
/robot2_controller
/robot_coordinator
```

### 2. Check Active Topics

```bash
ros2 topic list
```

Expected topics include:

```
/robot1/cmd_vel
/robot1/scan
/robot1/odom
/robot2/cmd_vel
/robot2/scan
/robot2/odom
/map
/tf
/tf_static
```

### 3. Check Transform Tree

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF showing the coordinate frame relationships.

### 4. Monitor Topic Activity

```bash
# Monitor robot command velocities
ros2 topic echo /robot1/cmd_vel

# Monitor lidar scans
ros2 topic echo /robot1/scan

# Monitor odometry
ros2 topic echo /robot1/odom
```

### 5. Check Node Information

```bash
# Get details about a specific node
ros2 node info /robot_coordinator

# Check node parameters
ros2 param list /robot1_controller
```

### 6. Test micro-ROS Agent Connectivity

```bash
# Check if micro-ROS agents are ready
ros2 service list | grep micro

# Monitor micro-ROS agent logs in the launch terminal
# Look for "Client connected" messages when ESP32s connect
```

---

## Component Status Indicators

### ✅ Healthy System Indicators:

- All nodes appear in `ros2 node list`
- No error messages in launch terminal
- Transform publishers are active
- Topics are being published (check with `ros2 topic hz <topic_name>`)
- micro-ROS agents show "waiting for clients" or "client connected"

### ❌ Problem Indicators:

- Error messages in launch terminal
- Missing nodes in `ros2 node list`
- No data on expected topics
- Transform lookup failures
- "process has died" messages

---

## Troubleshooting Common Issues

### Issue: Launch File Not Found

```bash
# Rebuild the package
colcon build --packages-select multirobot_nav
source install/local_setup.bash
```

### Issue: Module Import Errors

- Check `setup.py` entry points match actual Python file names
- Rebuild package after any changes

### Issue: Transform Errors

- Verify all static transform publishers are running
- Check transform tree with `view_frames`

### Issue: No ESP32 Connection

- Verify WiFi AP is working: `sudo systemctl status hostapd`
- Check DHCP service: `sudo systemctl status dnsmasq`
- Monitor micro-ROS agent logs for connection attempts

---

## Shutdown Procedure

1. **Stop the launch file**: Press `Ctrl+C` in the launch terminal
2. **Verify all processes stopped**: Check that all PIDs have terminated
3. **Clean shutdown**: All nodes should exit gracefully

---

## Additional Monitoring Commands

```bash
# Check system resource usage
htop

# Monitor network connections
ss -tuln

# Check ROS 2 daemon status
ros2 daemon status

# View system logs
journalctl -f
```

This guide should help you start the system correctly and verify that all components are working as expected.
