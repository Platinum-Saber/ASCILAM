# ASCILAM 
## (Autonomous Simultaneous Collaborative Integrated Localization And Mapping)
### Project Overview
Prototype implementation of a collaborative terrain mapping system using **modified M-Bots** as scouts and a **Raspberry Pi 4-powered robot** as the central coordinator. The scouts use LiDAR and odometry to map terrain, while the coordinator fuses their data into a global map in real time via **Wi-Fi using UDP**.

### System Architecture
<img width="3094" height="2004" alt="Communication Architecture" src="https://github.com/user-attachments/assets/2e22e173-c8b7-48bc-96d3-0e4930db88b8" />



### Core Objectives
- Utilize M-Bots with added LiDAR sensors to map 2D terrain.
- Set up UDP over Wi-Fi for real-time data transmission.
- Use the coordinator to localize and fuse scout maps into a global map.

---

### Hardware Components

| Component           | M-Bot Scouts (x2)                         | Coordinator Robot           |
|---------------------|-------------------------------------------|------------------------------|
| Base Platform       | Modified M-Bot                            | Custom Robot Chassis         |
| Controller          | M-Bot Controller + Wi-Fi Module (e.g. ESP32) | Raspberry Pi 4               |
| Sensors             | LiDAR (e.g., RPLiDAR A1), Wheel Encoders, IMU | None                         |
| Communication       | Wi-Fi + UDP                              | Wi-Fi + UDP    |
| Power Supply        | Rechargeable Battery                      | Power Bank / Battery Module  |

---

### Software Stack

| Task                | Tool/Package                               |
|---------------------|---------------------------------------------|
| OS                  | Custom FW (M-Bot) / Ubuntu 24.04 (Raspberry Pi) |
| Framework           | ROS 2 Jazzy (Coordinator)                  |
| SLAM & Mapping      | GMapping / Cartographer (Scouts)           |
| Localization        | `robot_localization` package               |
| Communication       | UDP over Wi-Fi                            |
| Map Merging         | `multirobot_map_merge` / Custom ROS Node   |
| Visualization       | RViz2                                      |

---


## Future Work
- Integrate autonomous navigation using global map.
- Enable camera/RGB-D input for semantic mapping.
- Add web dashboard for map monitoring.
