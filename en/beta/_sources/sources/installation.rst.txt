Installation
===============

The Supervisor is distributed as a debian package.
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

Copy and paste the following command in a terminal to install the Supervisor:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh)

This will install the Supervisor for the current distribution and architecture and the control panel.

At the end of the installation, the control panel will be started and you will be able to start the configuration of the supervisor.
To do so open a web browser and go to the following address: http://localhost:8000


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

