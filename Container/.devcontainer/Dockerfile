FROM osrf/ros:foxy-desktop

# Setup non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Add video group for camera access
RUN sudo usermod --append --groups video $USERNAME

# Update packages
RUN sudo apt update && sudo apt upgrade -y

# Base tools
RUN sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    htop \
    tree \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool

# Python packages for visualization and tools
RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    pyserial \
    scipy

# Set up ROS2 workspace for LD19 driver
RUN mkdir -p /home/$USERNAME/ldlidar_ros2_ws/src

WORKDIR /home/$USERNAME/ldlidar_ros2_ws/src

# Clone LD19 ROS2 Driver
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

WORKDIR /home/$USERNAME/ldlidar_ros2_ws

# Build driver
RUN . /opt/ros/foxy/setup.sh && colcon build

# Setup environment sourcing
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source ~/ldlidar_ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Final working directory
WORKDIR /home/$USERNAME/ldlidar_ros2_ws

# Default entrypoint
CMD [ "bash" ]
