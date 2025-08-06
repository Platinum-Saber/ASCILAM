FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Install additional packages for embedded systems development
RUN sudo apt-get install -y \
    build-essential \
    cmake \
    gdb \
    valgrind \
    curl \
    wget \
    vim \
    nano \
    tree \
    htop \
    python3 \
    python3-pip \
    python3-dev \
    gcc-multilib \
    g++-multilib \
    libc6-dev-i386 \
    software-properties-common \
    lsb-release \
    gnupg2

# Install useful Python packages for embedded development and ROS2
RUN pip3 install --no-cache-dir \
    matplotlib \
    numpy \
    scipy \
    pyserial \
    requests \
    setuptools==58.2.0

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
# Install essential simulation utilities
RUN sudo apt-get install -y ros-humble-joint-state-publisher-gui ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs

# Install RPLidar packages
RUN sudo apt-get install -y ros-humble-rplidar-ros

# Install ROS2 development tools
RUN sudo apt-get install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-vcstool

# Create ROS2 workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# Set up colcon completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Set working directory to the mounted workspace
WORKDIR /Container
