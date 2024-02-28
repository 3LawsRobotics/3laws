Getting Started
===============

This guide will help you to install and configure the Supervisor on your robot.

.. contents:: Table of Contents
   :depth: 2
   :local:


1. Installation
---------------

The first step is to install the Supervisor on your robot.

On your robot computer, open a terminal and run the following command:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh)

Some questions may be asked if the script is unable to find the ROS distribution or architecture of your computer.

Restart your terminal to complete the installation. If ROS is not sourced, run the following command:

.. code-block:: bash

  source /opt/ros/<DISTRO>/setup.sh

The Supervisor is now installed on your robot.

2. Configuration
----------------

After a successful installation, you will need to configure the Supervisor to match your robot.
For this use the Control Panel that started automatically after the installation or open a terminal and run the following command:

.. code-block:: bash

  sudo systemctl start lll_control_panel

By default the control panel is accessible to the following address: `http://localhost:8000`

Optionally, you can start a rosbridge server to connect the Control Panel to your robot:

.. code-block:: bash

  ros2 run rosbridge_server rosbridge_websocket

This will provide a websocket server at `ws://localhost:9090` that the control panel can connect to to get topics and services information.

Finally, open a web browser and go to the Control Panel address.
Follow the instructions to configure the Supervisor. You will need to provide the following information:

- **Robot ID**: The name of your robot
- **Credentials**: The 3Laws Robotics Inc. key provided to you on the purchase of the Supervisor.
- **Model**: The dynamic model of your robot.
- **Supervisor configuration**: The configuration of the Supervisor and control input interfaces.
- **State**: The state interface of your robot and safety constraints.
- **Perception**: The perception interface of your robot.

3. Launch
---------

Before starting the supervisor be sure to have your ROS environment correctly set up and sourced.

.. code-block:: bash

  source /opt/ros/<DISTRO>/setup.sh

To launch the Supervisor you can use the following command:

.. code-block:: bash

  ros2 launch lll_rdm rdm.launch.py

4. Monitor
----------

Thanks to the websocket server, you can monitor the Supervisor using the Control Panel.
Go to the Operations tab verify that the Supervisor is running and that the state and perception interfaces are correctly connected.



