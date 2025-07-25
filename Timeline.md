# Timeline

## 🟠 Phase 1: Setup & Communication (Week 1–2)
### Goal: Establish UDP communication between M-Bots and Coordinator Pi.
- [ ] Install ROS 2 Jazzy and Mosquitto on Raspberry Pi 4.
- [ ] Configure Wi-Fi and test UDP connections from M-Bots to Pi.
- [ ] Implement UDP streaming test (mock map and pose data).
- [ ] Coordinator logs incoming data via subscriber node.

## 🟡 Phase 2: M-Bot Mapping & Localization (Week 3–4)
### Goal: Enable M-Bots to build maps and estimate positions.
- [ ] Mount and configure LiDAR (e.g., RPLiDAR) on M-Bots.
- [ ] Collect odometry using encoders; estimate orientation via IMU.
- [ ] Fuse odometry and IMU using an EKF.
- [ ] Use GMapping or Cartographer for 2D SLAM per M-Bot.
- [ ] Publish pose and occupancy grid.

## 🟢 Phase 3: UDP Map Transmission & Merging (Week 5–6)
### Goal: Send and merge maps from M-Bots via MQTT to coordinator.
- [ ] Define `MapUpdate.msg` with grid + pose.
- [ ] Convert occupancy grid and pose to UDP format.
- [ ] Coordinator transforms and merges into a global map.
- [ ] Visualize global map in RViz2.

## 🔵 Phase 4: Field Testing & Optimization (Week 7–8)
### Goal: Evaluate merged map accuracy and system performance.
- [ ] Indoor test run: compare individual and merged maps.
- [ ] Tune UDP message rate and optimize payloads.
- [ ] Adjust map alignment and pose transforms.
