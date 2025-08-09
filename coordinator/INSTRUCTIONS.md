# ASCILAM Coordinator Folder Instructions

This guide explains how to use the `ld19_udp_viz` ROS2 package and the LD19 data simulation script to visualize UDP-transmitted LiDAR data in the ASCILAM coordinator workspace.

## 📁 Folder Structure

```
coordinator/
├── ld19_udp_viz/         # ROS2 package for UDP LiDAR visualization
├── sim_ld19p_udp.py      # Python script to simulate LD19 UDP data
└── INSTRUCTIONS.md       # This instruction file
```

## 🛠️ Prerequisites

- **ROS2 Foxy or later** installed and sourced

## 🚀 Step-by-Step Usage

## 🐢 Setting Up on Raspberry Pi 4 (ROS2 Foxy)

1. **Install and source ROS2 Foxy** on your Raspberry Pi 4. Follow the official ROS2 Foxy installation guide for Ubuntu 20.04.

2. **Create a ROS2 workspace:**

```bash
mkdir -p ~/ascilam_ws/src
cd ~/ascilam_ws/src
```

3. **Copy the `ld19_udp_viz` folder** from this repository into your workspace's `src` directory:

```bash
cp -r /path/to/ASCILAM/coordinator/ld19_udp_viz ~/ascilam_ws/src/
```

4. **Build the workspace:**

```bash
cd ~/ascilam_ws
colcon build --symlink-install
source install/setup.bash
```

5. **Continue with the steps below to run the simulation and visualization.**

The `sim_ld19p_udp.py` script simulates LD19 LiDAR data and transmits it over UDP.

**Usage Example:**

```bash
python sim_ld19p_udp.py --ip <target_ip> --port <target_port>
```

- Replace `<target_ip>` with the IP address where the ROS2 node is listening (often `127.0.0.1` for local testing).
- Replace `<target_port>` with the UDP port expected by the visualization node (default: `6001`).

```bash
python sim_ld19p_udp.py --ip 127.0.0.1 --port 6001
```

### 3. Visualize UDP Transmission with ROS2

Launch the visualization node from the `ld19_udp_viz` package:

```bash
ros2 launch ld19_udp_viz demo.launch.py
```

- This node will listen for UDP packets on the configured port and visualize the incoming LiDAR data.
- You may need to adjust the launch file or parameters for your network setup.

1. **Build the package:** `colcon build --symlink-install`
2. **Source the workspace:** `source install/setup.bash`
3. **Start the UDP simulation:** `python sim_ld19p_udp.py --ip 127.0.0.1 --port 6001`
4. **Launch the ROS2 visualization:** `ros2 launch ld19_udp_viz demo.launch.py`

## ⚙️ Troubleshooting

- **No visualization?**
  - Check that the UDP simulation script is running and sending data to the correct IP/port.
  - Verify that your firewall allows UDP traffic on the chosen port.
- **Build errors?**
  - Make sure your ROS2 environment is properly sourced before building.
- **Python errors?**
  - Ensure you are using Python 3.7 or newer.

## 📚 Additional Notes

- The `ld19_udp_viz` package is designed for rapid development; changes to Python code are reflected immediately when using `--symlink-install`.
- You can modify the simulation script to test different data rates, packet formats, or network conditions.

**Module:** LD19 UDP Visualization  
**Last Updated:** August 2025
