#!/bin/bash

# Arguments
WSL_IP=${1:-"host.docker.internal"}

# Configure enviroment variable
export DISPLAY="$WSL_IP:0.0"
echo "DISPLAY=$DISPLAY"

source /opt/ros/iron/setup.bash

# Launch PlotJuggler
ros2 run plotjuggler plotjuggler