#!/bin/bash
# Enhanced Multi-Robot System Startup Script
# Initializes poses and starts automatic frontier exploration

echo "=================================================="
echo "  ASCILAM Multi-Robot Dynamic Mapping System"
echo "  Starting with Automatic Frontier Exploration"
echo "=================================================="

# Set the workspace
cd ~/ASCILAM/multirobot_ws

# Source the workspace
source install/setup.bash

echo "Starting system components..."

# Start the main system in background
echo "Launching multi-robot system..."
ros2 launch multirobot_nav multi_robot_launch.py &
LAUNCH_PID=$!

# Wait for system to initialize
echo "Waiting for system initialization..."
sleep 10

# Initialize robot poses
echo "Setting initial robot poses..."
echo "  Robot1: Position (0,0,0), Orientation 0° (facing +X)"
echo "  Robot2: Position (1,0,0), Orientation 180° (facing -X)"
python3 ~/ASCILAM/config/initialize_robot_poses.py &
POSE_PID=$!

# Wait for pose initialization
sleep 5

echo "=================================================="
echo "System Status:"
echo "  ✓ Multi-robot SLAM with dynamic mapping"
echo "  ✓ Robot controllers with obstacle avoidance"  
echo "  ✓ Automatic frontier exploration"
echo "  ✓ Initial poses configured"
echo "=================================================="
echo ""
echo "Robots will automatically start exploring frontiers!"
echo "Monitor progress with:"
echo "  - rviz2 (for visualization)"
echo "  - ros2 topic echo /map (for map updates)"
echo "  - ros2 topic echo /robot1/goal_pose (for robot1 goals)"
echo "  - ros2 topic echo /robot2/goal_pose (for robot2 goals)"
echo ""
echo "Press Ctrl+C to stop the system"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping system..."
    if [ ! -z "$POSE_PID" ]; then
        kill $POSE_PID 2>/dev/null
    fi
    if [ ! -z "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null
    fi
    echo "System stopped."
    exit 0
}

# Set up signal handling
trap cleanup INT TERM

# Keep script running
wait $LAUNCH_PID