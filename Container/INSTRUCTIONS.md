# Embedded Systems Dev Container with ROS2 Foxy

This project is configured to run in a Docker dev container with Ubuntu 20.04, ROS2 Foxy, and desktop environment support for STL-19P LIDAR integration.

## What's Included

- **Base OS**: Ubuntu 20.04 LTS (via OSRF ROS2 Foxy Desktop image)
- **ROS2**: Foxy Fitzroy (LTS)
- **Desktop Environment**: VNC and web-based desktop access
- **Development Tools**:
  - GCC/G++ compiler toolchain
  - CMake build system
  - GDB debugger
  - Valgrind memory analyzer
  - Colcon build system for ROS2
- **VS Code Extensions**:
  - C/C++ IntelliSense and debugging
  - CMake Tools
  - Python support
  - ROS extension for VS Code
  - XML and YAML support for ROS2 files
- **Additional Packages**:
  - Python3 with pip
  - Common Python packages for embedded development (pyserial, numpy, matplotlib)
  - Multi-architecture compilation support
  - ROS2 development tools (rosdep, vcstool, colcon)
  - **LDLIDAR STL ROS2 packages** for STL-19P LIDAR sensor integration
- **Simulation Tools**:
  - Gazebo simulation environment
  - Joint state publisher GUI
  - XACRO for robot description## Getting Started

1. **Prerequisites**:

   - Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
   - Install [VS Code](https://code.visualstudio.com/)
   - Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. **Open in Dev Container**:

   - Open this folder in VS Code
   - When prompted, click "Reopen in Container" or use Command Palette: `Dev Containers: Reopen in Container`
   - Wait for the container to build (first time may take a few minutes)

3. **Access Desktop Environment**:

   - **Web Desktop**: Open http://localhost:6080 in your browser
   - **VNC Client**: Connect to `localhost:5901` with a VNC viewer
   - Use the desktop for GUI applications like Gazebo, RViz, etc.

4. **Verify Setup**:
   ```bash
   gcc --version
   python3 --version
   cmake --version
   ros2 --version
   colcon version-check
   ```

## ROS2 Usage

- **ROS2 Environment**: Automatically sourced in terminal sessions
- **Workspace**: A ROS2 workspace is created at `/home/ubuntu/ldlidar_ros2_ws/` with STL LIDAR drivers pre-installed
- **Build packages**: Use `colcon build` in the workspace
- **Source workspace**: `source install/setup.bash` after building

### Common ROS2 Commands:

```bash
# Check ROS2 installation
ros2 doctor

# List available nodes
ros2 node list

# Create a new package
ros2 pkg create --build-type ament_cmake my_package

# Build workspace
cd /home/ubuntu/ldlidar_ros2_ws
colcon build

# Run examples
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### STL-19P LIDAR Usage:

The container comes with pre-installed LDLIDAR STL ROS2 drivers supporting STL-19P, LD06, LD19, and STL-27L models.

**Available STL-19P Commands:**

```bash
# Set device permissions (run once after connecting LIDAR)
sudo chmod 777 /dev/ttyUSB0

# Launch STL-19P LIDAR node (basic)
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# Launch STL-19P with RViz2 visualization
ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py

# Manual RViz2 setup
rviz2
# Then open: ~/ldlidar_ros2_ws/src/ldlidar_stl_ros2/rviz2/ldlidar.rviz

# Check LIDAR data topics
ros2 topic list
ros2 topic echo /scan

# Monitor LIDAR node status
ros2 node list
ros2 node info /LD19
```

**STL-19P Configuration Parameters:**

```python
# Key parameters in ld19.launch.py:
{'product_name': 'LDLiDAR_LD19'}
{'port_name': '/dev/ttyUSB0'}          # Update based on your system
{'port_baudrate': 230400}              # STL-19P standard baud rate
{'laser_scan_dir': True}               # Counterclockwise scan
{'enable_angle_crop_func': False}      # Disable angle filtering
{'topic_name': 'scan'}                 # ROS2 topic name
{'frame_id': 'base_laser'}             # TF frame
```

**Troubleshooting STL-19P:**

```bash
# Check device detection
ls -l /dev/tty*

# Verify permissions
ls -l /dev/ttyUSB0

# Test serial communication
sudo dmesg | grep tty

# Check ROS2 environment
printenv | grep ROS
```

## Customization

- Modify `.devcontainer/Dockerfile` to add more packages
- Update `.devcontainer/devcontainer.json` to add VS Code extensions or settings
- The `postCreateCommand` runs after container creation for additional setup

## Port Forwarding

If your embedded system communicates over specific ports, add them to the `forwardPorts` array in `devcontainer.json`.

## Serial Device Access (Windows)

To use serial devices like STL-19P LIDAR in the container on Windows:

1. **Install usbipd-win**:

   ```powershell
   winget install usbipd
   ```

2. **List available USB devices**:

   ```powershell
   usbipd list
   ```

3. **Bind your STL-19P device** (run as Administrator):

   ```powershell
   usbipd bind --busid <BUSID>
   ```

4. **Attach to WSL** (each time you want to use it):

   ```powershell
   usbipd attach --wsl --busid <BUSID>
   ```

5. **Verify in container**:

   ```bash
   ls /dev/tty*
   # Should show /dev/ttyUSB0 or /dev/ttyACM0

   # Set permissions for STL-19P
   sudo chmod 777 /dev/ttyUSB0
   ```

## STL-19P LIDAR Specifications

- **Model**: LDROBOT STL-19P
- **Range**: 0.05m - 12m
- **Accuracy**: ±2cm
- **Scan Rate**: 10Hz
- **Angular Resolution**: 1°
- **Interface**: UART (230400 baud)
- **Power**: 5V via USB
- **Dimensions**: Compact form factor suitable for robotics applications

## Quick Start with STL-19P

1. **Connect Hardware**:

   - Connect STL-19P to USB port
   - Verify device recognition: `ls /dev/tty*`

2. **Set Permissions**:

   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```

3. **Launch LIDAR**:

   ```bash
   cd ~/ldlidar_ros2_ws
   source install/setup.bash
   ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
   ```

4. **Verify Data**:
   - Check RViz2 visualization
   - Monitor `/scan` topic
   - Verify 360° laser scan data

## Troubleshooting

- If the container fails to build, try: `Dev Containers: Rebuild Container`
- For permission issues, the container runs as the `vscode` user by default
- Check Docker Desktop is running and has sufficient resources allocated
