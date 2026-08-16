# build stage
FROM ros:jazzy-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=franka

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-jazzy-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

# create non-root user without sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

COPY --chown=$USERNAME:$USERNAME ./teleop_entrypoint.sh /teleop_entrypoint.sh
RUN chmod +x /teleop_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD [ "/bin/bash" ]
