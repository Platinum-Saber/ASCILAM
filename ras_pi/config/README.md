# ASCILAM Raspberry Pi Config Scripts Documentation

This document explains the purpose and usage of each script in the `ras_pi/config` folder for setting up your Raspberry Pi 4 (Ubuntu MATE 20.04) for multi-robot navigation with ROS 2 Foxy.

---

## 1. ros2_foxy_setup.sh

**Purpose:**
Installs ROS 2 Foxy and essential multi-robot navigation packages.

**Main Steps:**

- Updates package lists.
- Installs ROS 2 Foxy desktop and Python argcomplete.
- Installs colcon build tools and navigation packages (Nav2, SLAM Toolbox, robot localization, tf2 tools, etc.).

**Usage:**

```sh
chmod +x ros2_foxy_setup.sh
./ros2_foxy_setup.sh
```

---

## 2. setup_ros2_env.sh

**Purpose:**
Sets up the ROS 2 workspace, installs build tools, configures micro-ROS Agent, and creates a starter ROS 2 Python package for multi-robot navigation.

**Main Steps:**

- Installs ROS 2 build tools (`colcon`, `ament_cmake`, `rosdep`, etc.).
- Initializes and updates `rosdep` for dependency management.
- Adds ROS 2 Foxy environment sourcing to `~/.bashrc`.
- Creates the workspace directory (`multirobot_ws`).
- Clones and builds the micro-ROS setup.
- Installs workspace dependencies with `rosdep`.
- Builds the workspace and micro-ROS agent.
- Creates a starter ROS 2 Python package (`multirobot_nav`) with required dependencies and folders.

**Usage:**

```sh
chmod +x setup_ros2_env.sh
./setup_ros2_env.sh
```

---

## 3. setup_wifi_ap.sh

**Purpose:**
Configures the Raspberry Pi as a WiFi Access Point for multi-robot networking.

**Main Steps:**

- Installs `hostapd` and `dnsmasq` for AP and DHCP services.
- Stops services for configuration.
- Sets a static IP for `wlan0`.
- Configures `dnsmasq` for DHCP and static IP reservations for robots.
- Configures `hostapd` for WiFi AP settings (SSID, password, etc.).
- Sets up IP forwarding and NAT with `iptables`.
- Ensures `iptables` rules are restored on boot via `/etc/rc.local` (creates file if missing).
- Enables and starts `hostapd` and `dnsmasq` services.

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
