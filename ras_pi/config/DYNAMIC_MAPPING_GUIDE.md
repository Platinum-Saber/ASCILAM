# Dynamic Mapping System Guide

This guide explains the enhanced dynamic mapping capabilities for the ASCILAM multi-robot system.

## Overview

The dynamic mapping system extends the basic SLAM functionality to handle dynamic environments where obstacles can move or change over time. This includes:

- **Probabilistic Mapping**: Uses Bayesian updates instead of binary occupancy
- **Temporal Decay**: Reduces confidence in old observations
- **Dynamic Obstacle Detection**: Identifies frequently changing areas
- **Predictive Avoidance**: Predicts moving obstacle trajectories
- **Multi-Robot Coordination**: Handles interactions between robots

## Key Features

### 1. Probabilistic SLAM (`multirobot_slam.py`)
- **Bayesian Updates**: Each cell stores probability of occupancy (0.0-1.0)
- **Temporal Tracking**: Maintains observation history and timestamps
- **Decay Mechanism**: Old observations lose influence over time
- **Dynamic Detection**: Identifies cells with high variance in observations

### 2. Dynamic-Aware Navigation (`robot_controller.py`)
- **Obstacle Velocity Tracking**: Compares consecutive scans to detect movement
- **Predictive Collision Avoidance**: Estimates future obstacle positions
- **Emergency Stopping**: Immediate response to collision threats
- **Adaptive Speed Control**: Adjusts velocity based on dynamic obstacles

### 3. Parameter Tuning (`dynamic_tuning.py`)
- **Real-time Performance Monitoring**: Tracks computational load and detection rates
- **Automatic Parameter Suggestions**: Recommends adjustments based on performance
- **Metrics Logging**: Saves performance data for analysis

### 4. Comprehensive Testing (`test_dynamic_mapping_fixed.py`)
- **Multi-Phase Testing**: Tests different scenarios systematically
- **Performance Analysis**: Evaluates system effectiveness
- **Statistical Reporting**: Provides detailed performance metrics

## Configuration Parameters

### SLAM Parameters
```yaml
multi_robot_slam:
  ros__parameters:
    # Basic mapping
    map_resolution: 0.05      # Map cell size (meters)
    map_width: 2000          # Map width (cells)
    map_height: 2000         # Map height (cells)
    update_rate: 5.0         # Map publishing rate (Hz)
    
    # Dynamic environment parameters
    decay_rate: 0.95         # Temporal decay factor (0.9-0.99)
    min_observations: 3      # Min observations for dynamic detection
    temporal_window: 30.0    # Time window for observation history (seconds)
    dynamic_threshold: 0.3   # Variance threshold for dynamic detection
    
    # Probabilistic mapping
    occupied_threshold: 0.7  # Probability threshold for occupied cells
    free_threshold: 0.3      # Probability threshold for free cells
    prior_probability: 0.5   # Initial probability for unknown cells
```

### Controller Parameters
```yaml
robot_controller:
  ros__parameters:
    # Basic navigation
    linear_speed: 0.15       # Base forward speed (m/s)
    angular_speed: 0.3       # Base turning speed (rad/s)
    safe_distance: 0.4       # Static obstacle avoidance distance (m)
    goal_tolerance: 0.3      # Goal reaching tolerance (m)
    
    # Dynamic obstacle detection
    obstacle_threshold: 0.5  # Distance threshold for obstacle detection (m)
    safety_distance: 0.3     # Emergency stop distance (m)
    reaction_time: 0.2       # Time horizon for predictions (s)
    max_linear_velocity: 0.5 # Maximum forward speed (m/s)
    max_angular_velocity: 1.0 # Maximum turning speed (rad/s)
```

## Usage Instructions

### 1. Basic Operation

Launch the system with dynamic mapping:
```bash
cd /home/suhan/ROS_Jazzy/ascilam_ws
source install/setup.bash
ros2 launch ras_pi multi_robot_launch.py
```

### 2. Parameter Tuning

Run the tuning script to monitor and optimize performance:
```bash
# In a new terminal
ros2 run ras_pi dynamic_tuning.py
```

The tuner will:
- Monitor system performance in real-time
- Suggest parameter adjustments
- Save metrics to `/tmp/dynamic_mapping_metrics_*.json`

### 3. System Testing

Run comprehensive tests to validate functionality:
```bash
# In a new terminal
ros2 run ras_pi test_dynamic_mapping_fixed.py
```

Test phases:
1. **Static Mapping** (30s): Baseline performance with stationary robots
2. **Single Robot Movement** (60s): Basic dynamic detection
3. **Multi-Robot Movement** (90s): Coordination and interference handling
4. **Coordinated Exploration** (60s): Goal-based navigation
5. **Stress Test** (60s): Rapid movements and high dynamics

### 4. Monitoring and Visualization

Monitor the system using standard ROS tools:
```bash
# View map updates
ros2 topic echo /map

# Monitor robot scans
ros2 topic echo /robot1/scan
ros2 topic echo /robot2/scan

# Check robot commands
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot2/cmd_vel

# Visualize in RViz
rviz2
```

## Parameter Tuning Guidelines

### High Dynamic Environments
- **Decrease** `decay_rate` (0.90-0.93) - faster forgetting
- **Decrease** `temporal_window` (15-25s) - shorter memory
- **Decrease** `dynamic_threshold` (0.2-0.25) - more sensitive detection
- **Increase** `reaction_time` (0.3-0.4s) - longer prediction horizon

### Low Dynamic Environments  
- **Increase** `decay_rate` (0.96-0.99) - slower forgetting
- **Increase** `temporal_window` (40-60s) - longer memory
- **Increase** `dynamic_threshold` (0.4-0.5) - less sensitive detection
- **Decrease** `reaction_time` (0.1-0.2s) - shorter prediction horizon

### Performance Optimization
- **High CPU Usage**: Increase `decay_rate`, reduce `update_rate`
- **Low Detection Rate**: Decrease `dynamic_threshold`, increase `min_observations`
- **False Positives**: Increase `dynamic_threshold`, increase `min_observations`
- **Slow Response**: Decrease `reaction_time`, increase `max_angular_velocity`

## Troubleshooting

### Common Issues

1. **No Dynamic Detection**
   - Check if robots are actually moving
   - Verify scan data is being received
   - Lower `dynamic_threshold` parameter
   - Check `min_observations` setting

2. **High False Positive Rate**
   - Increase `dynamic_threshold`
   - Increase `min_observations`
   - Check sensor noise levels
   - Verify stable robot poses

3. **Poor Performance**
   - Monitor CPU usage with tuning script
   - Reduce `update_rate` if needed
   - Increase `decay_rate` for faster processing
   - Check network connectivity between robots

4. **Robots Not Avoiding Each Other**
   - Verify both robots are publishing scans
   - Check `safety_distance` parameter
   - Ensure obstacle detection is working
   - Verify coordinate frame transforms

### Debug Commands

```bash
# Check node status
ros2 node list
ros2 node info /multi_robot_slam
ros2 node info /robot1_controller
ros2 node info /robot2_controller

# Check topic publishing rates
ros2 topic hz /robot1/scan
ros2 topic hz /robot2/scan
ros2 topic hz /map

# Check parameter values
ros2 param list /multi_robot_slam
ros2 param get /multi_robot_slam decay_rate

# Monitor computational load
top -p $(pgrep -f "multi_robot_slam")
```

## Expected Performance

### Nominal Operation
- **Scan Rate**: 15-25 Hz combined from both robots
- **Map Update Rate**: 3-8 Hz depending on dynamics
- **CPU Usage**: <50% on Raspberry Pi 4
- **Detection Latency**: <200ms for dynamic obstacles
- **Memory Usage**: <512MB for 2000x2000 map

### Performance Indicators
- **Excellent**: >80% expected scan rate, active dynamic detection
- **Good**: 60-80% scan rate, some dynamic detection
- **Poor**: <60% scan rate, little to no dynamic detection

## Integration with Hardware

### ESP32 Configuration
- Ensure LiDAR data is published at consistent rates
- Verify odometry publication for both robots
- Check WiFi network stability
- Monitor power consumption during dynamic operations

### M-Bot Integration
- Verify motor response times are adequate
- Check that emergency stop commands are honored
- Ensure movement commands are executed promptly
- Monitor for mechanical issues affecting navigation

## Future Enhancements

Potential improvements for the dynamic mapping system:
- **Machine Learning**: Use ML for better dynamic object classification
- **Multi-Sensor Fusion**: Integrate additional sensors (cameras, IMU)
- **Cooperative Tracking**: Share dynamic object information between robots
- **Semantic Mapping**: Classify different types of dynamic objects
- **Path Planning**: Integration with advanced path planning algorithms