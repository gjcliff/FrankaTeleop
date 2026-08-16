FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=franka

ENV WS_DIR="/ros2_ws"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    cmake \
    git \
    software-properties-common \
    python3-pip

RUN apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-*

RUN apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

# create non-root user without sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

WORKDIR /ros2_ws/src/

COPY handcv/ ./handcv/
COPY hand_interfaces/ ./hand_interfaces/
COPY cv_franka_bridge/ ./cv_franka_bridge/
COPY franka_teleop/ ./franka_teleop/
COPY franka_teleop_bringup/ ./franka_teleop_bringup/

COPY --chown=$USERNAME:$USERNAME ./teleop_entrypoint.sh /teleop_entrypoint.sh
RUN chmod +x /teleop_entrypoint.sh

WORKDIR /ros2_ws/
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt --break-system-packages --ignore-installed

ENTRYPOINT ["/teleop_entrypoint.sh"]
CMD [ "/bin/bash" ]
