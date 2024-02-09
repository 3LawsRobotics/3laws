Installation
===============

The Control Panel is distributed as a debian package.
A package is available for each of the following configuration

+-----------------------+--------------+---------------------+---------------+
| Ubuntu Distribution   | ROS1 version |    ROS2 version     | Architecture  |
+=======================+==============+=====================+===============+
|        22.04          |     N/A      |     Humble/Iron     | amd64/arm64v8 |
+-----------------------+--------------+---------------------+---------------+
|        20.04          |     Noetic   |     Galactic/Foxy   | amd64/arm64v8 |
+-----------------------+--------------+---------------------+---------------+
|        18.04          |     Melodic  |          N/A        | amd64/arm64v8 |
+-----------------------+--------------+---------------------+---------------+

.. contents:: Table of Contents
   :depth: 2
   :local:


Quick Installation
------------------

Here is the one liner that will download a public script that will, based on your system download the right package.
**This script also install the RDM package.**

.. code-block:: bash

  sudo bash -c "$(wget -qO - https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh)"


Non Interactive Installation
----------------------------
If you want to install a specific package in a non interactive mode, here is the process:

- Download the script:

.. code-block:: bash

  wget https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh

- Make it executable

.. code-block:: bash

  chmod +x install.sh

- Run it with your arguments

.. code-block:: bash

  sudo ./install.sh [-hyf] [-r <ROS_DISTRO>] [-a <ARCH>] [-v <UBUNTU_VERSION>]

if ``-yf -r <ROS_DISTRO> -a <ARCH> -v <UBUNTU_VERSION>`` specified, the script is non interactive

Requirements
------------
- Python 3.10 or lower is required to run the Control Panel. The running script also use the library argparse.
- In order to provide realtime feedback on the supervisor and auto-completion for the configuration to the user, the control panel can use the ROS websocket bridge. This bridge is not installed by default.
Here is the documentation to install it: https://wiki.ros.org/rosbridge_suite.

