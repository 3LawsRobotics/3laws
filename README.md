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
- `-v <UBUNTU_DISTRO [18.04, 20.04, 22.04]>`
- `-r <ROS_DISTRO [iron, humble, galactic, foxy, noetic]>`
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

## Docker runtime environment

A sample Docker project is provided to assist users who prefer to install and execute Supervisor within a controlled environment. The project is not designed to meet any production-grade requirements, but rather to provide a starting point for further development.

From the root directory, run `./docker/build_docker.bash <ROS_DISTRO>` to create the Docker image. Replace `<ROS_DISTRO>` with either `humble` or `jazzy`. When the building is complete, run `./docker/run_docker.bash <ROS_DISTRO>` to start the container.

The start script creates a local directory (default at `~/.3laws`) and mounted in the container at (default) `/home/3laws/.3laws`. The shared volume is used to store Supervisor configurations so that all savings and progresses persists across restarts of the container.

In alternative to manually starting the container via the provided script, it is possible to call the script directly from a ROS 2 launch file:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    DOCKER_RUN_SCRIPT = os.path.join(
        <path/to/3laws/docker>,
        "run_docker.bash",
    )

    launchdesc = LaunchDescription(
        [
            ExecuteProcess(
                cmd=[DOCKER_RUN_SCRIPT],
                shell=True,
                output="screen",
                name="lll_supervisor_docker",
            )
        ]
    )

    return launchdesc

```

IMPORTANT: When running the container for the first time, the Supervisor node will fail as no configuration file exists yet. Configure Supervisor through the Control Panel and restart the container.

## Repo maintainer

Thomas Gurriet - tgurriet@3laws.io
