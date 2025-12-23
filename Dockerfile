FROM franka_ros2:latest

ENV WS_DIR="/ros2_ws"
ENV USERNAME="franka"
WORKDIR ${WS_DIR}

ARG USER_VID_ID=44

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
    ros-${ROS_DISTRO}-tf-transformations \
    && sudo rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=dialog

WORKDIR /ros2_ws/src/

COPY hand_interfaces/ ./hand_interfaces/
COPY cv_franka_bridge/ ./cv_franka_bridge/
COPY franka_teleop/ ./franka_teleop/
COPY teleop_entrypoint.sh ./teleop_entrypoint.sh

RUN sudo chmod +x teleop_entrypoint.sh

USER root

WORKDIR /ros2_ws/
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt --break-system-packages


ENTRYPOINT ["/ros2_ws/src/teleop_entrypoint.sh"]
