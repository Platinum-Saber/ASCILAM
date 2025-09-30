#!/bin/bash
# ASCILAM System Status Checker
# Shows current system status and port usage

echo "=================================================="
echo "  ASCILAM System Status"
echo "  $(date)"
echo "=================================================="

# Check network ports
echo "Network Port Status:"
echo "==================="

# Port 8888
PORT_8888_PID=$(lsof -ti:8888 2>/dev/null)
if [ ! -z "$PORT_8888_PID" ]; then
    PORT_8888_PROC=$(ps -p $PORT_8888_PID -o comm= 2>/dev/null)
    echo "Port 8888: OCCUPIED by PID $PORT_8888_PID ($PORT_8888_PROC)"
else
    echo "Port 8888: FREE"
fi

# Port 8889  
PORT_8889_PID=$(lsof -ti:8889 2>/dev/null)
if [ ! -z "$PORT_8889_PID" ]; then
    PORT_8889_PROC=$(ps -p $PORT_8889_PID -o comm= 2>/dev/null)
    echo "Port 8889: OCCUPIED by PID $PORT_8889_PID ($PORT_8889_PROC)"
else
    echo "Port 8889: FREE"
fi

echo ""

# Check ROS2 processes
echo "ROS2 Process Status:"
echo "==================="

check_process() {
    local process_name=$1
    local display_name=$2
    local pids=$(pgrep -f "$process_name" 2>/dev/null)
    
    if [ ! -z "$pids" ]; then
        echo "$display_name: RUNNING (PIDs: $pids)"
        return 0
    else
        echo "$display_name: NOT RUNNING"
        return 1
    fi
}

# Check main processes
RUNNING_COUNT=0

if check_process "micro_ros_agent.*8888" "micro-ROS Agent Robot1"; then
    ((RUNNING_COUNT++))
fi

if check_process "micro_ros_agent.*8889" "micro-ROS Agent Robot2"; then  
    ((RUNNING_COUNT++))
fi

if check_process "multirobot_slam" "Multi-Robot SLAM"; then
    ((RUNNING_COUNT++))
fi

if check_process "robot_controller.*robot1" "Robot1 Controller"; then
    ((RUNNING_COUNT++))
fi

if check_process "robot_controller.*robot2" "Robot2 Controller"; then
    ((RUNNING_COUNT++))
fi

if check_process "robot_coordinator" "Robot Coordinator"; then
    ((RUNNING_COUNT++))
fi

echo ""

# Overall system status
echo "System Status Summary:"
echo "====================="

if [ $RUNNING_COUNT -eq 0 ]; then
    echo "Status: STOPPED"
    echo "All processes are stopped. System is ready to start."
elif [ $RUNNING_COUNT -ge 5 ]; then
    echo "Status: FULLY RUNNING"
    echo "All core components are active."
else
    echo "Status: PARTIALLY RUNNING"  
    echo "Some components are running ($RUNNING_COUNT/6)."
    echo "This might indicate a startup issue or partial shutdown."
fi

echo ""

# Check for common issues
echo "Diagnostics:"
echo "============"

# Check if ports are blocked
if [ ! -z "$PORT_8888_PID" ] || [ ! -z "$PORT_8889_PID" ]; then
    if [ $RUNNING_COUNT -lt 2 ]; then
        echo "⚠ WARNING: Ports are occupied but agents not detected"
        echo "   This usually means previous processes weren't cleaned up properly"
        echo "   Run: ./cleanup_system.sh"
    fi
fi

# Check workspace
if [ ! -d "~/ASCILAM/multirobot_ws" ]; then
    echo "⚠ WARNING: Workspace directory not found at ~/ASCILAM/multirobot_ws"
fi

# Check if system can be started
if [ $RUNNING_COUNT -eq 0 ]; then
    if [ -z "$PORT_8888_PID" ] && [ -z "$PORT_8889_PID" ]; then
        echo "✓ READY: System can be started with ./start_exploration.sh"
    else
        echo "✗ BLOCKED: Ports occupied, run ./cleanup_system.sh first"
    fi
elif [ $RUNNING_COUNT -ge 5 ]; then
    echo "✓ ACTIVE: System is running normally"
else
    echo "⚠ UNSTABLE: Partial operation detected, consider restart"
fi

echo ""
echo "Commands:"
echo "========="
echo "Start system:   ./start_exploration.sh"
echo "Stop system:    ./cleanup_system.sh"
echo "Check status:   ./status_check.sh"