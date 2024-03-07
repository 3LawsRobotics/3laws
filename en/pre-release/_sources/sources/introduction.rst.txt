Introduction
============

The 3Laws Robotics Supervisor is a software package that provides a control filter to ensure safety without compromising performance.
Additionally, the Supervisor contains a diagnostic module that monitors multiple metrics that indicate safety and performance.

Multiple ROS version are supported:

+-----------------------+--------------+---------------------+
| Ubuntu Distribution   | ROS1 version |    ROS2 version     |
+=======================+==============+=====================+
|        22.04          |     N/A      |     Humble/Iron     |
+-----------------------+--------------+---------------------+
|        20.04          |     Noetic   |     Galactic/Foxy   |
+-----------------------+--------------+---------------------+
|        18.04          |     Melodic  |          N/A        |
+-----------------------+--------------+---------------------+


Architecture
------------

The Supervisor is composed of 3 main modules:

- **RAM**: The Run-time Assurance Module is the core of the Supervisor. It is a filter that operates at the controller level to ensures that the directional commands sent to the robot do not violate proximity constraints relative to laserscan data. This component runs in near-real-time on the robot.
- **RDM**: The Robot Diagnostic Module is a collection of metrics calculated to indicate the robot health and safety. Data is aggregated on the robot by this module and is then shared to a cloud server for display.
- **Control Panel**: The Control Panel is an optional web-based application that guides the user through the configuration of the Supervisor. Based on the data provided by the user, it creates (or updates) a configuration file used by the Supervisor's various capabilities.  The Control Panel has some abilities to visualize the robot's safety.  This component does not need to be active (or running) during operation of the robot, except as desired for visualizing the metrics available through it.   Once the Supervisor is configured through the Control Panel, turning off the lll_control_panel.service is a reasonable step.

.. image:: data/architecture.png
   :width: 800px
   :alt: Architecture schema

Supervisor interfaces
---------------------

The 3Laws Robotics Supervisor currently requires that the robot is using the Robot Operating System (`ROS <http://www.ros.org>`_).
The Supervisor is a ROS node that subscribes to the robot's state, sensors, planning, computational measures, perception, and control commands.  If the RTA capability is enabled, Supervisor will also publish safety-filtered control commands to the robot's actuators.


RunTime Assurance Module (RAM)
--------------------------------

The RunTime Assurance Module (RAM), also referred to as **Copilot**, is a filter that operates at the control level. It is designed to ensure that the robot's control commands are safe and do not violate safety constraints related to proximity to points measured by a laser scanner.
Based on formal mathematical proven methods, RAM is able to prevent the robot from colliding while still allowing the robot to reach maximum performance when
the system is far from any obstacles in its current travel direction. 

This ability allows development of the robot's control and planning algorithms without worrying about collision avoidance.

RAM uses basic kinematic and dynamic models for the robot in order to
predict potential collisions.  Supervisor currently supports differential-drive (able to rotate-in-place and translate), single-track steered, and omni-directional (able to translate sideways in addition to rotations and forward/back motion) vehicles.


Robot Diagnostic Module (RDM)
-----------------------------

The Robot Diagnostic Module (RDM) is a diagnostic tool that monitors and compute metrics about the robot health and safety.
The computed metrics are published on ROS topics that.  High frequency versions of the metrics can be used to monitor the robot's health and safety in real-time.  In other words, these metrics are available to other processes on the robot, that can then use the information to make decisions.

The metrics are also summarized and sent to a cloud database for display and analysis. 3LawsRobotics cloud dashboards help diagnose issues the robot might have. These Dashboards are made using `Grafana <https://grafana.com/grafana/>`_.

Configuration
-------------
For effective operation, the Supervisor needs to be configured.
The `Control Panel <control_panel/index.html>`_ will help with Supervisor configuration.
The Control Panel will also help visualize operation of the Supervisor's Run-time Assurance (RTA) in action.

