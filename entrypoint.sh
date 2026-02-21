#!/bin/bash
set -e

# Source standard ROS setup
source /opt/ros/noetic/setup.bash

# Source our workspace setup
if [ -f "/ros_ws/devel/setup.bash" ]; then
    source "/ros_ws/devel/setup.bash"
fi

# Execute the command passed to the container
exec "$@"