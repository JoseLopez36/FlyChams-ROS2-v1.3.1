#!/bin/bash

# Get ROS2 workspace directory
ROS_WS="$HOME/FlyingChameleons/ros2_ws"

# Source ROS2 workspace
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
else
  echo "Error: ROS workspace not found at $ROS_WS" >&2
  exit 1
fi

# Launch Airsim settings node with parameters
ros2 run flychams_bringup airsim_settings_node --ros-args \
  -p config_source_file:="$HOME/FlyingChameleons/config/Configuration.xlsx" \
  -p airsim_settings_destination_file:="$HOME/FlyingChameleons/config/settings.json"