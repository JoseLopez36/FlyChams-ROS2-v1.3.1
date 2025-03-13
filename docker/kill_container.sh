#!/bin/bash
CONTAINER_NAME=flychams-ros2-container

RUNNING_CONTAINER=$(sudo docker ps -q -f "name=${CONTAINER_NAME}")

if [ -n "$RUNNING_CONTAINER" ]; then
    echo "Stopping container: ${CONTAINER_NAME}"
    sudo docker stop ${RUNNING_CONTAINER}
    echo "Removing container: ${CONTAINER_NAME}"
    sudo docker rm ${RUNNING_CONTAINER}
else
    echo "No containers running with the name: ${CONTAINER_NAME}"
fi
