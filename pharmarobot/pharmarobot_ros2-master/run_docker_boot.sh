#!/bin/bash

# Script to build the Docker image and run a container

# --- Configuration ---
IMAGE_NAME="tiagobarrosisr/pharmarobot"
IMAGE_TAG="latest"  # Change this to "latest" if you want to use the latest build
CONTAINER_NAME="pharma_container"


# Host directory to map to /workspace in the container.
# This script assumes it's located in the same directory as your Dockerfile.
HOST_WORKSPACE_DIR_RAW="$(pwd)" # Using "." means the directory where the script is run.
                           # "$(pwd)" would also work but "." is often simpler here.

# Resolve to the canonical, absolute path, following all symlinks
HOST_WORKSPACE_DIR=$(readlink -f "${HOST_WORKSPACE_DIR_RAW}")
if [ ! -d "${HOST_WORKSPACE_DIR}" ]; then
    echo "Error: Resolved HOST_WORKSPACE_DIR '${HOST_WORKSPACE_DIR}' is not a directory."
    echo "       (Original path was: '${HOST_WORKSPACE_DIR_RAW}', resolved to '${HOST_WORKSPACE_DIR}')"
    exit 1
fi

# --- Run Docker Container ---

echo "Attempting to stop and remove existing container named '${CONTAINER_NAME}' (if any)..."
docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "Running Docker container '${CONTAINER_NAME}' from image '${IMAGE_NAME}:${IMAGE_TAG}'...\n"
echo "Host workspace mounted to /ros_ws: ${HOST_WORKSPACE_DIR}"


docker run --rm --tty \
--privileged \
--network=host \
--device=/dev/ttyUSB0 \
--device=/dev/input/js0 \
--name "${CONTAINER_NAME}" \
--env ROS_HOSTNAME=localhost \
--env ROS_MASTER_URI=http://localhost:11311 \
"${IMAGE_NAME}:${IMAGE_TAG}" \
bash -c "source /opt/ros/humble/setup.bash && \
            colcon build --symlink-install && \
           source install/setup.bash && \
           ros2 launch teleop_pharma teleop_launch.xml && \
           exec bash"


echo "Container '${CONTAINER_NAME}' has exited."
