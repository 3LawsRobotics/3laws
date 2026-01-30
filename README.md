# 3Laws Repository

<br>
<p align="center">
  <a href="https://github.com/3LawsRobotics/3laws">
    <img src="media/logo.png" alt="Logo" width="640" height="139">
  </a>

  <h3 align="center">3Laws' Public Repo</h3>

  <p align="center">
    <a href="https://github.com/3LawsRobotics/3laws/"><strong>Explore the repo»</strong></a>
    <br />
    <a href="https://docs.3laws.io/"><strong>Explore the docs»</strong></a>
    <br />
  </p>
</p>

## Introduction

This repository purpose is to offer an easy access to the binary files of the 3Laws products.
The first public release of the [**Supervisor**](#Robot-diagnostic-module-installation)
is already available at a beta state. To get more information about this product, please contact [support@3lawsrobotics.com](support@3lawsrobotics.com)

Documentation and Tutorial will soon be available.

## Forum

Issues, questions, announcements and general discussions can be created and found at: [https://github.com/3LawsRobotics/3laws/discussions](https://github.com/3LawsRobotics/3laws/discussions).

## Releases

Release changelog and files can be found at: [https://github.com/3LawsRobotics/3laws/releases](https://github.com/3LawsRobotics/3laws/releases).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html), although API compatibility is only guaranteed from version `1.0.0` onward.

## Supervisor installation

### Interactive Package installation

```bash
bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh)
```

### Specific package

You can add arguments after the command to specify the wanted ros and ubuntu version, the desired CPU architecture and non interactive arguments like Always Yes and Force.

- `-a <ARCH [amd64, arm64]>`
- `-v <UBUNTU_DISTRO [ubuntu18.04, ubuntu20.04, ubuntu22.04, debian12]>`
- `-r <ROS_DISTRO [jazzy, iron, humble, kilted]>`
- `-f <Force even if versions doesn't match>`
- `-y <Always yes>`

```bash
bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh) [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>]
```

### Non interactive

Download the package:

```bash
wget https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh
```

Make it executable:

```bash
chmod +x install.sh
```

Run it with your arguments:

```bash
sudo ./install.sh [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>]
```

if `-yf -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION>` specified, the script is non interactive

As an example:

```bash
sudo ./install.sh -yf -r foxy -a arm64 -v 20.04
```

## Supervisor uninstall:

This command will fully remove supervisor from your computer

```bash
bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/uninstall.sh)
```

## ROS2 install from source

For distributions without available ROS2 packages (like debian), you can use the following scrip to build and install ROS2 in a way that is compatible with 3Laws Supervisor: [install_ros2.sh](https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install_ros2.sh).

To use it, you must first install a couple of dependencies:

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  apt-transport-https \
  apt-utils \
  ca-certificates \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  wget
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  build-essential \
  git \
  git-lfs
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-colcon-zsh \
  python3-dev \
  python3-flake8 \
  python3-matplotlib \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml
colcon metadata update
sudo rosdep init
rosdep update
```

You can then download and run the script:

```bash
bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install_ros2.sh)
```

## Repo maintainer

Thomas Gurriet - tgurriet@3laws.io
