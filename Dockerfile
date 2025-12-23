FROM franka_ros2:latest

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN sudo apt-get update \
    && sudo apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
    python3-pip \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-rqt-graph \
    && sudo rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=dialog

WORKDIR /ros2_ws/src/

COPY hand_interfaces/ ./hand_interfaces/
COPY cv_franka_bridge/ ./cv_franka_bridge/
COPY franka_teleop/ ./franka_teleop/

WORKDIR /ros2_ws/
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt --break-system-packages
