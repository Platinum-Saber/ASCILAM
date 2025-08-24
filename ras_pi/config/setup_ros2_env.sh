set -e
# Script to set up ROS 2 environment, micro-ROS Agent, and create a ROS 2 package
# Install ROS 2 build tools and ament_cmake
echo "Installing ROS 2 build tools and ament_cmake..."
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential ros-foxy-ament-cmake

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
	echo "Initializing rosdep..."
	sudo rosdep init
fi
rosdep update
# For Raspberry Pi 4 with Ubuntu MATE 20.04 and ROS 2 Foxy

set -e

# 1.6 Setup ROS2 Environment
echo "Adding ROS 2 Foxy setup to ~/.bashrc..."
grep -qxF 'source /opt/ros/foxy/setup.bash' ~/.bashrc || echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# Create workspace
echo "Creating ROS 2 workspace..."
mkdir -p ~/ASCILAM/multirobot_ws/src
cd ~/ASCILAM/multirobot_ws

# 1.7 Install micro-ROS Agent
echo "Cloning micro-ROS setup..."

# Install workspace dependencies
echo "Installing workspace dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

cd ~/ASCILAM/multirobot_ws/src
git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git || true
cd ~/ASCILAM/multirobot_ws
colcon build
source install/local_setup.bash

# Create and build micro-ROS agent workspace
echo "Setting up micro-ROS agent..."
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# 2. Create ROS2 Package
echo "Creating ROS 2 Python package: multirobot_nav..."
cd ~/ASCILAM/multirobot_ws/src
ros2 pkg create --build-type ament_python multirobot_nav --dependencies rclpy sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs laser_geometry || true
cd multirobot_nav
mkdir -p launch config

echo "Setup complete."
