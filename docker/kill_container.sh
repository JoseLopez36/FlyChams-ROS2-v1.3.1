#!/bin/bash
CONTAINER_NAME=cosys-airsim-ros2-container

RUNNING_CONTAINER=$(docker ps -q -f "name=${CONTAINER_NAME}")

if [ -n "$RUNNING_CONTAINER" ]; then
    echo "Stopping container: ${CONTAINER_NAME}"
    docker stop ${RUNNING_CONTAINER}
    echo "Removing container: ${CONTAINER_NAME}"
    docker rm ${RUNNING_CONTAINER}
else
    echo "No containers running with the name: ${CONTAINER_NAME}"
fi