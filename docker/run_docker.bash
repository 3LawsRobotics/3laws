#!/bin/bash

set -e # Exit on error, print commands, and treat unset variables as an error

TAG="$1"
IMAGE_NAME="lll-supervisor-runtime"

# Get all tags for the image
tags=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "^${IMAGE_NAME}:" | cut -d: -f2)

# Check if TAG exists in tags
if echo "$tags" | grep -q "^${TAG}$"; then
    IMAGE_TAG=$TAG
else
    echo "Selected tag '$TAG' not found. Available tags are:"
    echo "$tags"
    exit 1
fi


LOCAL_USER_HOME=/home/$USER
LOCAL_LLL_CONFIG_DIR="$LOCAL_USER_HOME/.3laws"
DOCKER_HOME=/home/3laws
DOCKER_LLL_CONFIG_DIR="$DOCKER_HOME/.3laws"
IMAGE_NAME="lll-supervisor-runtime"

# Check if Docker image exists
if ! docker image inspect $IMAGE_NAME:$IMAGE_TAG >/dev/null 2>&1; then
  echo "Docker image $IMAGE_NAME:$IMAGE_TAG not found. Run build_docker.bash first to create image."
  exit 1
fi

# Check if local ~/.3laws directory exists, if not create it
if [ ! -d "$LOCAL_LLL_CONFIG_DIR" ]; then
  mkdir -p "$LOCAL_LLL_CONFIG_DIR"
  chown -R "$(id -u)":"$(id -u)" "$LOCAL_LLL_CONFIG_DIR"
  chmod -R u+rwX "$LOCAL_LLL_CONFIG_DIR"
  echo "Created directory $LOCAL_LLL_CONFIG_DIR"
fi

docker run --privileged --rm -it \
  --name lll-supervisor \
  --net=host \
  -v "$LOCAL_LLL_CONFIG_DIR":"$DOCKER_LLL_CONFIG_DIR" \
  $IMAGE_NAME:$IMAGE_TAG
