# Use the NVIDIA CUDA base image with cuDNN
FROM nvidia/cuda:12.6.1-devel-ubuntu22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential packages and tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    lsb-release \
    gnupg2 \
    sudo \
    vim \
    nano \
    net-tools \
    python3-pip \
    python3-dev \
    python3-venv \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Configure locale settings
RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# Install ROS 2 Humble
RUN apt update && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt upgrade -y && \
    apt install -y ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Navigation (Nav2) and other ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-rviz2 \
    ros-humble-image-pipeline \
    ros-humble-geometry2 \
    ros-humble-rclcpp \
    && rm -rf /var/lib/apt/lists/*

# # Install ZED SDK 4.1.2 for Ubuntu 22.04
# RUN cd /tmp && \
#     curl -L https://download.stereolabs.com/zedsdk/4.1.2/ubuntu22.04 -o zed_sdk_4.1.2_ubuntu22.04.run && \
#     chmod +x zed_sdk_4.1.2_ubuntu22.04.run && \
#     bash ./zed_sdk_4.1.2_ubuntu22.04.run || (echo "Failed to install ZED SDK" && exit 1)

# Update package lists and install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    # Add the universe repository
    add-apt-repository universe && \
    # Update package lists again after adding the repository
    apt-get update && \
    # Install zstd
    apt-get install -y --no-install-recommends zstd && \
    cd /tmp && \
    # Download and install the ZED SDK
    curl -L https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -o zed_sdk_4.1.2_ubuntu22.04.run && \
    chmod +x zed_sdk_4.1.2_ubuntu22.04.run && \
    ./zed_sdk_4.1.2_ubuntu22.04.run -- silent || (echo "Failed to install ZED SDK" && exit 1) && \
    # Clean up
    rm zed_sdk_4.1.2_ubuntu22.04.run && \
    apt-get clean && rm -rf /var/lib/apt/lists/*



# Install other required libraries
RUN apt-get update && \
    apt-get install -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    && pip3 install --upgrade \
    setuptools \
    wheel

WORKDIR /home

# Default command
CMD ["bash"]
