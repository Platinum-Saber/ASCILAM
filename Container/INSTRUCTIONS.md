# Embedded Systems Dev Container with ROS2 Humble

This project is configured to run in a Docker dev container with Ubuntu 22.04, ROS2 Humble, and desktop environment support.

## What's Included

- **Base OS**: Ubuntu 22.04 LTS (via OSRF ROS2 Humble Desktop image)
- **ROS2**: Humble Hawksbill (LTS)
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
  - **RPLidar ROS2 packages** for LIDAR sensor integration
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
- **Workspace**: A ROS2 workspace is created at `/home/ubuntu/ros2_ws/`
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
cd /home/ubuntu/ros2_ws
colcon build

# Run examples
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### RPLidar Usage:
- Refer [Official RPLidar documentation](https://github.com/Slamtec/rplidar_ros/tree/ros2) to install the ROS2 packages.

```bash
# Launch RPLidar node
ros2 launch rplidar_ros rplidar_a1_launch.py

# Launch RPLidar with custom parameters
ros2 launch rplidar_ros rplidar_a2m8_launch.py

# View laser scan data in RViz
ros2 run rviz2 rviz2
```

## Customization

- Modify `.devcontainer/Dockerfile` to add more packages
- Update `.devcontainer/devcontainer.json` to add VS Code extensions or settings
- The `postCreateCommand` runs after container creation for additional setup

## Port Forwarding

If your embedded system communicates over specific ports, add them to the `forwardPorts` array in `devcontainer.json`.

## Serial Device Access (Windows)

To use serial devices like RPLidar in the container on Windows:

1. **Install usbipd-win**:

   ```powershell
   winget install usbipd
   ```

2. **List available USB devices**:

   ```powershell
   usbipd list
   ```

3. **Bind your device** (run as Administrator):

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
   ```

## Troubleshooting

- If the container fails to build, try: `Dev Containers: Rebuild Container`
- For permission issues, the container runs as the `vscode` user by default
- Check Docker Desktop is running and has sufficient resources allocated
