ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ros:$ROS_DISTRO-ros-core
FROM $BASE_IMAGE

## Sup options
ARG SUP_VERSION=1.4.0
ARG ASSET_ID
ARG ASSET_NAME

## User options
ARG USER_NAME=3laws
ARG LLL_USER_ID
ARG LLL_GROUP_ID
ARG LLL_DIALOUT_GID
ARG LLL_INPUT_GID

## Install some packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    jq \
    software-properties-common \
    ros-$ROS_DISTRO-rosbridge-server

# Install libstd++13
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libstdc++-13-dev

# To allow sourcing
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

## Create user
RUN userdel ubuntu || true
RUN groupadd -f --gid ${LLL_DIALOUT_GID} dialout
RUN groupadd -f --gid ${LLL_INPUT_GID} input
RUN groupadd -f --gid 1000 $USER_NAME-1000
RUN groupadd -f --gid 1001 $USER_NAME-1001
RUN groupadd -f --gid 2000 $USER_NAME-2000
RUN groupadd -f --gid 999 $USER_NAME-999
RUN groupadd -f --gid 118 $USER_NAME-118
RUN groupadd -f --gid $LLL_GROUP_ID $USER_NAME
RUN useradd  -ms /bin/bash $USER_NAME --uid $LLL_USER_ID -g $LLL_GROUP_ID -G 1000,1001,2000,999,118
RUN echo "$USER_NAME:$USER_NAME" | chpasswd
RUN adduser $USER_NAME sudo

# Remove need for password with sudo command
RUN echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

# Add user to sudo and root groups
RUN usermod -aG sudo $USER_NAME
RUN usermod -aG root $USER_NAME

# Switch to user
USER $USER_NAME
ENV HOME=/home/$USER_NAME
WORKDIR $HOME
RUN touch ".sudo_as_admin_successful"


## Get Supervisor
ENV GH_REPO="https://api.github.com/repos/3LawsRobotics/3laws"

WORKDIR /tmp
ARG CURL_ARGS="-LJO#"
ARG GH_ASSET="$GH_REPO/releases/assets/$ASSET_ID"
RUN curl $CURL_ARGS -s -H 'Accept: application/octet-stream' "$GH_ASSET"

## Install Supervisor
RUN sudo apt install -f ./"$ASSET_NAME" -y --no-install-recommends

WORKDIR $HOME
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Add entrypoint
COPY entrypoint.sh /home/$USER_NAME/entrypoint.sh
ENTRYPOINT ["/home/3laws/entrypoint.sh"]
