#!/bin/bash
CONTAINER_NAME=cosys-airsim-ros2-container

# Check if container is already running
if docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container is already running, opening a new terminal"
    docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/iron/setup.bash; exec bash"
else
    echo "Container is not running, starting it"
    docker run --rm -it \
    -v ${FLYCHAMS_PATH}:/home/testuser/FlyingChameleons \
    --name ${CONTAINER_NAME} \
    cosys-airsim-ros2:latest bash
fi