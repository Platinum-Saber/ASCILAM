# Automatic Frontier Exploration Setup Guide

## Overview

Your ASCILAM system is now configured for **automatic frontier exploration** with proper initial robot positioning. The robots will start at defined positions and automatically explore the environment by finding and navigating to unexplored areas (frontiers).

## Initial Robot Configuration

### Robot 1
- **Position**: (0, 0, 0) - Origin point
- **Orientation**: 0° (facing +X direction)
- **Role**: Primary explorer

### Robot 2  
- **Position**: (1, 0, 0) - 1 meter along X-axis
- **Orientation**: 180° (facing -X direction)
- **Role**: Secondary explorer

## How It Works

### 1. **Automatic Frontier Detection**
- The SLAM system continuously analyzes the map
- Identifies boundaries between known free space and unknown areas
- Groups nearby frontier points into clusters
- Filters clusters by size and distance from origin

### 2. **Intelligent Goal Assignment**
- Coordinator assigns the closest frontier to each robot
- Maintains minimum distance between robot goals
- Reassigns goals when robots reach their destinations
- Handles goal timeouts for stuck robots

### 3. **Dynamic Obstacle Avoidance**
- Each robot avoids static and dynamic obstacles
- Predicts moving obstacle trajectories
- Implements emergency stopping for collision prevention
- Adapts navigation speed based on obstacle proximity

## Quick Start

### Option 1: Enhanced Startup Script (Recommended)
```bash
cd /home/suhan/ROS_Jazzy/ascilam_ws/src/ASCILAM/ras_pi/config
./start_exploration.sh
```

### Option 2: Manual Launch
```bash
cd /home/suhan/ROS_Jazzy/ascilam_ws
source install/setup.bash

# Launch the system
ros2 launch ras_pi multi_robot_launch.py

# In another terminal, initialize poses
python3 ras_pi/config/initialize_robot_poses.py
```

## System Monitoring

### View Map Updates
```bash
ros2 topic echo /map
```

### Monitor Robot Goals
```bash
# Robot 1 goals
ros2 topic echo /robot1/goal_pose

# Robot 2 goals  
ros2 topic echo /robot2/goal_pose
```

### Check Robot Positions
```bash
ros2 topic echo /robot1/odom
ros2 topic echo /robot2/odom
```

### Monitor System Status
```bash
ros2 node list
ros2 topic list
```

## Configuration Parameters

### Exploration Behavior
```yaml
robot_coordinator:
  auto_start_exploration: true      # Start exploration automatically
  frontier_min_size: 5             # Minimum frontier cluster size
  frontier_cluster_distance: 1.0   # Frontier grouping distance (m)
  goal_assignment_interval: 3.0    # How often to assign new goals (s)
  max_exploration_range: 10.0      # Maximum exploration distance (m)
  min_robot_distance: 1.5          # Minimum distance between robots (m)
```

### Robot Navigation
```yaml
robot_controller:
  auto_explore: true               # Enable automatic exploration
  exploration_goal_timeout: 30.0  # Goal timeout (s)
  linear_speed: 0.15              # Base movement speed (m/s)
  angular_speed: 0.3              # Base turning speed (rad/s)
  goal_tolerance: 0.3             # Distance to consider goal reached (m)
```

## Expected Behavior

### Startup Sequence (0-10 seconds)
1. System components initialize
2. Initial poses are set
3. SLAM begins mapping immediate surroundings
4. Dynamic obstacle detection activates

### Early Exploration (10-60 seconds)
1. Coordinator detects initial frontiers
2. Assigns closest frontiers to each robot
3. Robots navigate to assigned goals
4. Map expands as new areas are discovered

### Active Exploration (60+ seconds)
1. Continuous frontier detection and assignment
2. Robots automatically get new goals upon completion
3. Dynamic obstacle avoidance maintains safety
4. Map quality improves with temporal decay

### Completion
1. When 90% of reachable area is explored
2. System automatically stops exploration
3. Robots halt at their current positions

## Troubleshooting

### Robots Not Moving
- Check that both robots are connected via WiFi
- Verify M-Bot serial communication
- Ensure goals are being assigned: `ros2 topic echo /robot1/goal_pose`

### No Frontiers Detected
- Check map is being built: `ros2 topic echo /map`
- Verify LiDAR data: `ros2 topic echo /robot1/scan`
- Increase `max_exploration_range` parameter

### Robots Getting Stuck
- Check `exploration_goal_timeout` setting
- Verify obstacle avoidance parameters
- Monitor emergency stops in logs

### Poor Exploration Coverage
- Adjust `frontier_min_size` (smaller = more sensitive)
- Reduce `frontier_cluster_distance` for finer detection
- Increase `max_exploration_range` for wider coverage

## Performance Optimization

### For Small Spaces
```yaml
max_exploration_range: 5.0
frontier_min_size: 3
goal_assignment_interval: 2.0
```

### For Large Spaces  
```yaml
max_exploration_range: 20.0
frontier_min_size: 8
goal_assignment_interval: 5.0
```

### For Dynamic Environments
```yaml
decay_rate: 0.90
reaction_time: 0.1
safety_distance: 0.5
```

## Advanced Features

### Manual Goal Override
You can still send manual goals that will override automatic exploration:
```bash
ros2 topic pub /robot1/goal_pose geometry_msgs/PoseStamped "header: {frame_id: 'map'} pose: {position: {x: 2.0, y: 1.0}}"
```

### Real-time Parameter Tuning
```bash
# Change exploration range
ros2 param set /robot_coordinator max_exploration_range 15.0

# Adjust goal timeout
ros2 param set /robot1_controller exploration_goal_timeout 45.0
```

### Performance Monitoring
Use the dynamic tuning script for real-time performance analysis:
```bash
python3 ras_pi/config/dynamic_tuning.py
```

## Visualization with RViz2

Launch RViz2 to visualize the exploration:
```bash
rviz2
```

Add these displays:
- **Map** (`/map`) - Shows explored areas
- **LaserScan** (`/robot1/scan`, `/robot2/scan`) - LiDAR data  
- **PoseStamped** (`/robot1/goal_pose`, `/robot2/goal_pose`) - Current goals
- **Odometry** (`/robot1/odom`, `/robot2/odom`) - Robot positions

## Safety Features

- **Collision Avoidance**: Robots avoid each other and obstacles
- **Emergency Stopping**: Immediate halt for collision threats  
- **Goal Timeouts**: Prevents robots from getting permanently stuck
- **Distance Limits**: Constrains exploration to reasonable range
- **Dynamic Detection**: Adapts to moving obstacles in real-time

The system is now fully configured for autonomous multi-robot exploration!