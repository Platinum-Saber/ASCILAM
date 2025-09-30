#!/bin/bash
# Enhanced Multi-Robot System Startup Script
# Initializes poses and starts automatic frontier exploration

echo "=================================================="
echo "  ASCILAM Multi-Robot Dynamic Mapping System"
echo "  Starting with Automatic Frontier Exploration"
echo "=================================================="

# Function to cleanup any existing processes
cleanup_existing_processes() {
    echo "Cleaning up any existing processes..."
    
    # Kill any existing micro-ros agents on ports 8888 and 8889
    echo "Checking for existing micro-ROS agents..."
    
    # Find and kill processes using port 8888
    PORT_8888_PID=$(lsof -ti:8888 2>/dev/null)
    if [ ! -z "$PORT_8888_PID" ]; then
        echo "Killing existing process on port 8888 (PID: $PORT_8888_PID)"
        kill -9 $PORT_8888_PID 2>/dev/null
        sleep 1
    fi
    
    # Find and kill processes using port 8889
    PORT_8889_PID=$(lsof -ti:8889 2>/dev/null)
    if [ ! -z "$PORT_8889_PID" ]; then
        echo "Killing existing process on port 8889 (PID: $PORT_8889_PID)"
        kill -9 $PORT_8889_PID 2>/dev/null
        sleep 1
    fi
    
    # Kill any existing ROS2 launch processes
    echo "Cleaning up existing ROS2 processes..."
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "multi_robot_launch" 2>/dev/null
    pkill -f "multirobot_slam" 2>/dev/null
    pkill -f "robot_controller" 2>/dev/null
    pkill -f "robot_coordinator" 2>/dev/null
    pkill -f "micro_ros_agent" 2>/dev/null
    
    # Wait for processes to clean up
    sleep 3
    echo "Process cleanup completed."
}

# Cleanup existing processes first
cleanup_existing_processes

# Set the workspace
cd ~/ASCILAM/multirobot_ws

# Source the workspace
source install/setup.bash

echo "Starting system components..."

# Verify ports are free before starting
echo "Verifying ports are available..."
PORT_8888_CHECK=$(lsof -ti:8888 2>/dev/null)
PORT_8889_CHECK=$(lsof -ti:8889 2>/dev/null)

if [ ! -z "$PORT_8888_CHECK" ]; then
    echo "ERROR: Port 8888 is still occupied by PID $PORT_8888_CHECK"
    echo "Please run ./cleanup_system.sh first"
    exit 1
fi

if [ ! -z "$PORT_8889_CHECK" ]; then
    echo "ERROR: Port 8889 is still occupied by PID $PORT_8889_CHECK"
    echo "Please run ./cleanup_system.sh first"
    exit 1
fi

echo "✓ Ports 8888 and 8889 are free"

# Start the main system in background
echo "Launching multi-robot system..."
ros2 launch multirobot_nav multi_robot_launch.py &
LAUNCH_PID=$!

# Check if launch was successful
sleep 5
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "ERROR: Failed to start the main system"
    exit 1
fi

# Wait for system to initialize
echo "Waiting for system initialization..."
sleep 10

# Initialize robot poses
echo "Setting initial robot poses..."
echo "  Robot1: Position (0,0,0), Orientation 0° (facing +X)"
echo "  Robot2: Position (1,0,0), Orientation 180° (facing -X)"
python3 ~/ASCILAM/config/set_pose/initialize_robot_poses.py &
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
    
    # Kill pose initialization process
    if [ ! -z "$POSE_PID" ]; then
        kill $POSE_PID 2>/dev/null
        echo "Stopped pose initializer"
    fi
    
    # Kill main launch process
    if [ ! -z "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null
        echo "Stopped main launch process"
    fi
    
    # Force cleanup of all related processes
    echo "Performing thorough cleanup..."
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "multi_robot_launch" 2>/dev/null
    pkill -f "multirobot_slam" 2>/dev/null
    pkill -f "robot_controller" 2>/dev/null
    pkill -f "robot_coordinator" 2>/dev/null
    pkill -f "micro_ros_agent" 2>/dev/null
    
    # Clean up ports
    PORT_8888_PID=$(lsof -ti:8888 2>/dev/null)
    if [ ! -z "$PORT_8888_PID" ]; then
        kill -9 $PORT_8888_PID 2>/dev/null
    fi
    
    PORT_8889_PID=$(lsof -ti:8889 2>/dev/null)
    if [ ! -z "$PORT_8889_PID" ]; then
        kill -9 $PORT_8889_PID 2>/dev/null
    fi
    
    echo "System fully stopped and cleaned up."
    exit 0
}

# Set up signal handling
trap cleanup INT TERM

# Keep script running
wait $LAUNCH_PID