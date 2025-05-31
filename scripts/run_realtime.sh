#!/bin/bash
# Wrapper script to run ROS2 node with real-time priority and proper environment

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace environment
WORKSPACE_DIR="/home/ros/sigyn_testicle_twister_ws"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Set ROS_DOMAIN_ID if not already set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Run the command with real-time priority and CPU affinity
exec chrt -f 50 taskset -c 2 "$@"
