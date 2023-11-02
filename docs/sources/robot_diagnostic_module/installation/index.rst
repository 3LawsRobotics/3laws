Installation
===============

The Robot Diagnostic Module is distributed as a debian package.
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

Here is the one liner that will download a public script that will, based on your system download the right package.

.. code-block:: bash

  sudo bash -c "$(wget -qO - https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh)"


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

