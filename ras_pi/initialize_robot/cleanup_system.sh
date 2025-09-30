#!/bin/bash
# ASCILAM System Cleanup Script
# Stops all running processes and cleans up ports

echo "=================================================="
echo "  ASCILAM System Cleanup"
echo "  Stopping all processes and cleaning ports"
echo "=================================================="

# Function to safely kill processes
safe_kill() {
    local process_name=$1
    local pids=$(pgrep -f "$process_name" 2>/dev/null)
    
    if [ ! -z "$pids" ]; then
        echo "Stopping $process_name processes: $pids"
        echo $pids | xargs kill -TERM 2>/dev/null
        sleep 2
        
        # Force kill if still running
        local remaining_pids=$(pgrep -f "$process_name" 2>/dev/null)
        if [ ! -z "$remaining_pids" ]; then
            echo "Force stopping remaining $process_name processes: $remaining_pids"
            echo $remaining_pids | xargs kill -9 2>/dev/null
        fi
    fi
}

# Stop ROS2 processes
echo "Stopping ROS2 processes..."
safe_kill "ros2 launch"
safe_kill "multi_robot_launch"
safe_kill "multirobot_slam"
safe_kill "robot_controller" 
safe_kill "robot_coordinator"
safe_kill "micro_ros_agent"
safe_kill "initialize_robot_poses"

# Clean up specific ports
echo "Cleaning up network ports..."

# Port 8888 (Robot 1 micro-ROS agent)
PORT_8888_PID=$(lsof -ti:8888 2>/dev/null)
if [ ! -z "$PORT_8888_PID" ]; then
    echo "Freeing port 8888 (PID: $PORT_8888_PID)"
    kill -9 $PORT_8888_PID 2>/dev/null
    sleep 1
else
    echo "Port 8888 is free"
fi

# Port 8889 (Robot 2 micro-ROS agent)  
PORT_8889_PID=$(lsof -ti:8889 2>/dev/null)
if [ ! -z "$PORT_8889_PID" ]; then
    echo "Freeing port 8889 (PID: $PORT_8889_PID)"
    kill -9 $PORT_8889_PID 2>/dev/null
    sleep 1
else
    echo "Port 8889 is free"
fi

# Clean up any remaining Python processes related to the project
echo "Cleaning up Python processes..."
pkill -f "dynamic_tuning.py" 2>/dev/null
pkill -f "test_dynamic_mapping" 2>/dev/null
pkill -f "initialize_robot_poses.py" 2>/dev/null

# Wait for cleanup to complete
sleep 2

# Verify cleanup
echo ""
echo "Verification:"
echo "=============="

# Check ports
PORT_8888_CHECK=$(lsof -ti:8888 2>/dev/null)
PORT_8889_CHECK=$(lsof -ti:8889 2>/dev/null)

if [ -z "$PORT_8888_CHECK" ]; then
    echo "✓ Port 8888 is free"
else
    echo "⚠ Port 8888 still occupied by PID: $PORT_8888_CHECK"
fi

if [ -z "$PORT_8889_CHECK" ]; then
    echo "✓ Port 8889 is free"  
else
    echo "⚠ Port 8889 still occupied by PID: $PORT_8889_CHECK"
fi

# Check for remaining ROS processes
REMAINING_ROS=$(pgrep -f "ros2|multirobot|micro_ros" 2>/dev/null)
if [ -z "$REMAINING_ROS" ]; then
    echo "✓ No ROS2 processes running"
else
    echo "⚠ Remaining ROS processes: $REMAINING_ROS"
fi

echo ""
echo "=================================================="
echo "Cleanup completed!"
echo "You can now safely restart the system with:"
echo "  ./start_exploration.sh"
echo "=================================================="