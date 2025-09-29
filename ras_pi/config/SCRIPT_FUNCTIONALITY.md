# ASCILAM Raspberry Pi Config Scripts: Functionality Documentation

## 1. ros2_foxy_setup.sh

**Purpose:**
Automates installation of ROS 2 Foxy and essential multi-robot navigation packages on Raspberry Pi 4 (Ubuntu MATE 20.04).

**Functionality:**

- Updates the system's package list.
- Installs ROS 2 Foxy desktop (core ROS 2 tools and GUI).
- Installs `python3-argcomplete` for command-line auto-completion in ROS 2.
- Installs `python3-colcon-common-extensions` for building ROS 2 workspaces.
- Installs navigation and multi-robot packages:
  - `ros-foxy-nav2-bringup`, `ros-foxy-navigation2`, `ros-foxy-nav2-map-server` (Nav2 stack)
  - `ros-foxy-slam-toolbox` (SLAM)
  - `ros-foxy-robot-localization` (sensor fusion)
  - `ros-foxy-tf2-tools` (transform tools)
- Prints status messages for each step.

**Usage:**

```sh
chmod +x ros2_foxy_setup.sh
./ros2_foxy_setup.sh
```

---

## 2. setup_ros2_env.sh

**Purpose:**
Sets up a ROS 2 workspace, installs build tools, configures micro-ROS Agent, and creates a starter ROS 2 Python package for multi-robot navigation.

**Functionality:**

- Installs ROS 2 build tools (`colcon`, `ament_cmake`, `rosdep`, etc.).
- Initializes and updates `rosdep` for dependency management.
- Adds ROS 2 Foxy environment sourcing to `~/.bashrc` for every new shell.
- Creates the workspace directory (`multirobot_ws`) in the ASCILAM folder.
- Clones the micro-ROS setup repository for microcontroller communication.
- Installs workspace dependencies using `rosdep`.
- Builds the workspace with `colcon build`.
- Sources the workspace environment for ROS 2 commands.
- Sets up and builds the micro-ROS agent workspace.
- Creates a starter ROS 2 Python package (`multirobot_nav`) with dependencies and folders for launch/config files.
- Prints status messages for each step.

**Usage:**

```sh
chmod +x setup_ros2_env.sh
./setup_ros2_env.sh
```

---

## 3. setup_wifi_ap.sh

**Purpose:**
Configures the Raspberry Pi as a WiFi Access Point for multi-robot networking.

**Functionality:**

- Installs `hostapd` (WiFi AP daemon) and `dnsmasq` (DHCP/DNS server).
- Stops both services to allow configuration changes.
- Sets a static IP for the wireless interface (`wlan0`).
- Configures `dnsmasq` for DHCP, DNS, and static IP reservations for robots.
- Configures `hostapd` for WiFi AP settings (SSID, password, channel, etc.).
- Sets the hostapd daemon config file path.
- Enables IP forwarding in the system for NAT.
- Configures `iptables` rules for NAT (internet sharing).
- Ensures `iptables` rules are restored on boot by creating and editing `/etc/rc.local` if needed.
- Enables and starts `hostapd` and `dnsmasq` services.
- Prints status messages for each step.

**Usage:**

```sh
chmod +x setup_wifi_ap.sh
sudo ./setup_wifi_ap.sh
```

---

**Note:**

- Run these scripts in order for a complete setup.
- Reboot after running `setup_wifi_ap.sh` to apply network changes.
- If you move your workspace, rebuild it with `colcon build` to avoid path issues.
