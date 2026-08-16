#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
