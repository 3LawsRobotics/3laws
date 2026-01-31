#!/bin/bash

DOCKER_IMAGE_NAME="lll-supervisor-runtime"

# Get ROS_DISTRO from first argument, error if not provided
if [ -z "$1" ]; then
  echo "Usage: $0 <ROS_DISTRO>. Specify jazzy|humble"
  exit 1
fi

# Check if provided ROS_DISTRO is either "jazzy" or "humble"
if [ "$1" != "jazzy" ] && [ "$1" != "humble" ]; then
  echo "Error - supported ROS_DISTRO values are: jazzy|humble"
  exit 1
fi
TAG=$1

GH_REPO="https://api.github.com/repos/3LawsRobotics/3laws/releases/latest"
PACKAGE_NAME="lll-supervisor-full-${TAG}"
REGEX_QUERY="${PACKAGE_NAME}_[0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]\+_amd64_ubuntu[0-9]\+.[0-9]\+"

# Read asset tags.
RESPONSE=$(curl -s -H "application/vnd.github+json" $GH_REPO)
ASSET_NAME=$(echo "$RESPONSE" | grep -o "name.:.\+${REGEX_QUERY}.deb" | cut -d ":" -f2- | cut -d "\"" -f2-)
ASSET_ID=$(echo "$RESPONSE" | grep -C3 "name.:.\+$REGEX_QUERY" | grep -w id | tr : = | tr -cd '[[:alnum:]]=' | cut -d'=' -f2-)

echo "Building Docker image $DOCKER_IMAGE_NAME:$TAG with asset $ASSET_NAME (id: $ASSET_ID)"

docker build --rm \
  --build-arg ROS_DISTRO="$TAG" \
  --build-arg ASSET_ID="$ASSET_ID" \
  --build-arg ASSET_NAME="$ASSET_NAME" \
  -t "$DOCKER_IMAGE_NAME:$TAG" \
  -f "$(pwd)/Dockerfile" .
