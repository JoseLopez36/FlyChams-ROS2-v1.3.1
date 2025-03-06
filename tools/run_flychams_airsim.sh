#!/bin/bash
set -e  # Exit on error

# Arguments   
LAUNCH_RVIZ=${1:-"false"}
RECORD=${2:-"false"}
RVIZ_CONFIG=${3:-"default.rviz"}
WSL_IP=${4:-"host.docker.internal"}

# Configure enviroment variable DISPLAY
export DISPLAY="$WSL_IP:0.0"
echo "DISPLAY=$DISPLAY"

# Get ROS2 workspace directory
ROS_WS="$FLYCHAMS_ROS2_PATH/ros2_ws"

# Source ROS2 workspace
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
else
  echo "Error: ROS workspace not found at $ROS_WS" >&2
  exit 1
fi

# Launch FlyChams with AirSim
echo "Launching FlyingChameleons with AirSim"
ros2 launch flychams_bringup flychams_airsim.launch.py &
FLYCHAMS_PID=$!

# Launch bag record conditionally
if [ "$RECORD" = "true" ]; then
  ros2 launch flychams_bringup bag_record.launch.py &
  BAG_RECORD_PID=$!
fi

# Launch RViz conditionally
if [ "$LAUNCH_RVIZ" = "true" ]; then
  sleep 3  # Brief delay
  echo "Launching RViz2 with config: $RVIZ_CONFIG"
  ros2 launch flychams_bringup rviz.launch.py config:="$RVIZ_CONFIG" 2>&1 | tee rviz.log
else
  echo "RViz launch skipped"
  wait  # Keep script alive until background processes finish
fi

# Cleanup on exit
trap "kill $AIRSIM_PID $FLYCHAMS_PID 2>/dev/null" EXIT