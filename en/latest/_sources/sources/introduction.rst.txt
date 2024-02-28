Introduction
============

The 3Laws Robotics supervisor is a software package that provides a control filter to insure safety without compromising performance.
In addition, the supervisor contains a diagnostic module to monitor relevant metrics for safety and performance.

In order to make the configuration and the usage of the supervisor easier, we provide a control panel, that will guide you through the configuration of the supervisor and help you visualize the supervisor's safety filter in action.

**Installation:**

- :doc:`installation`

**Quick Start:**

- :doc:`quick_start`


ROS dependencies
----------------

The 3Laws Robotics supervisor is designed to work with ROS.
The supervisor is a ROS node that subscribes to the robot's state, perception and control commands, and publishes the filtered control commands to the robot's actuators.
Also the supervisor can subscribes to the robot's sensors and planning topics and publishes the diagnostic information.

Multiple ROS version are supported. Here is a list of the tested versions:

+-----------------------+--------------+---------------------+
| Ubuntu Distribution   | ROS1 version |    ROS2 version     |
+=======================+==============+=====================+
|        22.04          |     N/A      |     Humble/Iron     |
+-----------------------+--------------+---------------------+
|        20.04          |     Noetic   |     Galactic/Foxy   |
+-----------------------+--------------+---------------------+
|        18.04          |     Melodic  |          N/A        |
+-----------------------+--------------+---------------------+


RealTime Assurance Module (RAM)
--------------------------------

The RealTime Assurance Module (RAM) is a filter at the control level. It is designed to ensure that the robot's control commands are safe and do not violate any safety constraints.
Based on formal mathematical proven methods, the RAM is able to guarantee that the robot will not violate any safety constraints, while still allowing the robot to perform at its best.

This ability can be used to safely develop your robot's control and planning algorithms without the need to worry about safety.
With the supervisor, focus on your core value and let the supervisor take care of the safety.

The RAM needs a basics kinematic and dynamic model of the robot and safety constraints on state in order to perform the right filtering.
3LawsRobotics knows that it is not always easy to describe a model of the robot which is why the supervisor is designed to be easily configurable and adaptable to the robot's model.

Robot Diagnostic Module (RDM)
-----------------------------

The Robot Diagnostic Module (RDM) is a diagnostic tool that monitors and compute metrics about the robot health and safety.
The computed metrics are published on ROS topics and can be used to monitor the robot's health and safety in real-time but can also be send to a data base for further analysis.
3LawsRobotics has developed a set of Dashboards to help developers quickly diagnose issues the robot might have. These Dashboards are made using `Grafana <https://grafana.com/grafana/>`_.

Configuration
-------------
The Supervisor require a bit of configuration to work properly.
Knowing that this process can be a bit complex, 3LawsRobotics created a `control panel <control_panel/index.html>`_ to help you configure the supervisor.

The control panel is a web application that will guide you through the configuration of the supervisor.
It will ask you for the robot's model, the safety constraints and the control commands topics.
Also, the control panel will help you visualize the supervisor's safety filter in action.

In order to successfully identify the user, the configuration requires a token that 3LawsRobotics will provide you with.
