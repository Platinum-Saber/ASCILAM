#!/bin/bash
# Script to set up ROS 2 Foxy and multi-robot packages on Raspberry Pi 4 with Ubuntu MATE 20.04

set -e

# Update package lists
sudo apt update

# Install ROS 2 Foxy desktop and required Python package
echo "Installing ROS 2 Foxy desktop and python3-argcomplete..."
sudo apt install -y ros-foxy-desktop python3-argcomplete

# Install XRCE-DDS middleware
# echo "Installing ros-foxy-rmw-micro-xrce-dds..."
# sudo apt install -y ros-foxy-rmw-micro-xrce-dds

# Install additional packages for multi-robot
echo "Installing colcon common extensions..."
sudo apt install -y python3-colcon-common-extensions

echo "Installing Nav2 and related packages..."
sudo apt install -y ros-foxy-nav2-bringup \
    ros-foxy-navigation2 \
    ros-foxy-nav2-map-server \
    ros-foxy-slam-toolbox \
    ros-foxy-robot-localization \
    ros-foxy-tf2-tools

echo "All packages installed successfully."
