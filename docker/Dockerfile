ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ros:$ROS_DISTRO-ros-core
FROM $BASE_IMAGE

## Sup options
ARG ASSET_ID
ARG ASSET_NAME

## Install some packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    software-properties-common \
    screen \
    vim \
    ros-$ROS_DISTRO-rosbridge-server

## Get Supervisor
ENV GH_REPO="https://api.github.com/repos/3LawsRobotics/3laws"

## Install Supervisor
WORKDIR /tmp
ARG CURL_ARGS="-LJO#"
ARG GH_ASSET="$GH_REPO/releases/assets/$ASSET_ID"
RUN curl $CURL_ARGS -s -H 'Accept: application/octet-stream' "$GH_ASSET"
RUN sudo apt install -f ./"$ASSET_NAME" -y --no-install-recommends

## Add entrypoint
WORKDIR /
COPY entrypoint.sh /root/entrypoint.sh
ENTRYPOINT ["/root/entrypoint.sh"]
