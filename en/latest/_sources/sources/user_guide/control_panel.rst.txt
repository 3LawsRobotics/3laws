Control Panel
##############

.. contents:: Table of Contents
  :local:

Overview
=========

The control panel is a web-based configuration and monitoring tool for the Supervisor. It is composed of three main sections:

  - Configuration: This section allows the user to configure the Supervisor's parameters.

  - Operations: This section shows the current state of the Supervisor and the robot.

  - Diagnostics: This section provides access to diagnostic visualization tools.

Each section is accessed by clicking on the corresponding tab in the Control Panel:

.. image:: ../data/cp_nav_header.png
  :align: center
  :width: 800px
  :alt: Control Panel tabs.

|

A section is also divided into sub-sections, which are accessed by clicking on the corresponding sub-tabs:

.. image:: ../data/cp_tabs.png
  :align: center
  :width: 800px
  :alt: Control Panel sub-tabs.

|

In addition, the navigation bar gives you access to other information, namely the documentation, instruction on how to launch the Supervisor ROS node, and a rosbridge connection status.


Rosbridge connection
=====================

The Control Panel is most effective when used with a ROS bridge server. This server provides a websocket interface to ROS topics. With this interface, the Control Panel can retrieve information about the **available topics** and help you complete the configuration of the Supervisor.

To install a Rosbridge server (where <distro> is replaced with the version of ROS on the system), run:

.. code-block:: bash

  sudo apt-get install ros-<distro>-rosbridge-server

The Rosbridge server can be started with the following command:

.. tabs::
   .. tab:: ROS1
     .. code-block:: bash

       roslaunch rosbridge_server rosbridge_websocket.launch

   .. tab:: ROS2
     .. code-block:: bash

       ros2 launch rosbridge_server rosbridge_websocket_launch.xml

This will provide a websocket server at **`ws://localhost:9090`**. The Control Panel can connect in order to retrieve topic and service information.

The navigation bar of the control panel shows the status of the rosbridge server connection:

.. image:: ../data/navigation_bar_rosbridge.png
  :width: 800px
  :alt: Control Panel NavBar with ros bridge connected.

A **green** status icon indicates that the Control Panel is connected to the Rosbridge server, while a **red** icon indicates that the Control Panel is not connected.

By clicking on the icon, you can set the IP address and port of the Rosbridge server to which the Control Panel should connect.

.. image:: ../data/rosbridge_connection_modal.png
  :align: center
  :width: 600px
  :alt: Rosbridge connection dialog.

|

.. note::

  During the configuration process, the Control Panel can be used without a Rosbridge server, but the autocompletion for topics, the operation tab and some diagnostic tools will not be available.

.. _control_panel_config:

Supervisor Configuration
========================

The Supervisor's configurable fields are available through a series of pages in the Control Panel. Each of the following sections corresponds to a tab in the Control Panel. The tabs are:

The **Save** button on each page of the Control Panel should be pressed to record the current page in the *supervisor.yaml* file before moving on to another page.

Throughout this documentation, a red asterisk (*) indicates a *required* field.

.. toctree::
  :maxdepth: 1

  1. General configuration <configuration/credentials>
  2. Model configuration <configuration/robot_model>
  3. Localization configuration <configuration/localization>
  4. Perception configuration <configuration/perception>
  5. Control configuration <configuration/control>
  6. Supervisor configuration <configuration/supervisor>


.. note::

  The configuration is saved in the `~/.3laws/config/supervisor.yaml` file.

.. warning::

  The Control Panel has a topic autocompletion capability that depends on the Rosbridge server. The robot's stack (without Supervisor) needs to be running and publishing topics for this feature's proper operation.

.. _control_panel_ops:

Control Panel's Operations Page
===============================

Status
------

.. image:: ../data/cp_operation.png
  :align: center
  :width: 800px
  :alt: Operations page showing a configured robot.

|

In the image above, the Supervisor is operational and all the components necessary for proper collision avoidance are configured to be active. The Diagnostic module is also operational and no issue as been detected (RDM pill in green).

If some of the components are not yet operational, the Control Panel will show the boxes in different colors (orange for warning, red for errors).
To get additional information about the error, you can check the logs by clicking on the button above the RDM/RAM pill. You can also click on the Status on top of the box to get details on the error source.

.. image:: ../data/cp_operation_error.png
  :align: center
  :width: 800px
  :alt: Operations page showing a configured robot that does not yet have sensor or planning data.


The above image shows a situation where all components are down. We have clicked on the detail status of the Motion Planner components.


A **blue** box indicates that the component is not yet operational and still initializing.

The lower section of the panel show logs and strip charts. The categories that are currently displayed include:

- **Latest logs**: Shows the most recent event from the RAM's Fault Manager.

- **State Safeness**: the Barrier Function (safety-related metric) value. When this value goes to zero or below zero, the system is evaluated as being in a collision state.

- **Input Modification**: When this value is zero, the Run-time Assurance Module is passing the input from the planner through to the lower-level UNchanged. That is, the filtering is in passive mode. When this value is non-zero, the Run-time Assurance Module is actively modifying the commanded input.

.. warning::

  In order to make this page work, the Rosbridge server needs to be active, and the Control Panel must connect to it.

.. _control_panel_viz:

Visualization
-------------

Visualization of the Runtime Assurance Module (RAM) metadata is available through `Rviz <https://wiki.ros.org/rviz>`_. This tab of the control panel will provide you with a link to download an Rviz configuration file consistent with the current Supervisor configuration.


Control Panel's Diagnostic Page
===============================

The Diagnostic page provides a summary of the diagnostic metrics computed by the Supervisor. Presented as plots or status timeline, these metrics are useful for debugging and tuning the Supervisor's parameters.
Divided into sections, the Diagnostic page shows the following metrics:

- System metrics
- Odometry metrics
- Dynamics metrics

System Health metrics
---------------------

.. image:: ../data/cp_system_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's system diagnostic metrics.

|

The System Metrics section shows the following metrics:

- **System health tree**
- **CPU load**
- **Memory usage**
- **Disk usage**
- **Network usage**
- **Process counts**

Control tracking health metrics
-------------------------------

.. image:: ../data/cp_diagnostic_control_tracking.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's system diagnostic metrics.

|

This tab show for each controller the tracking error as a norm and per component (if bounds are specified)

Odometry Metrics
----------------

.. image:: ../data/cp_odometry_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's odometry diagnostic metrics.

|

The Odometry Metrics section shows the following metrics if configured:

- **Odometry General**: This let you visualize a component of the odometry across all available ROS odometry topics.
- **Diagnostic**: This let you visualize a timeline of the status of each odometry. The timeline can have 3 colors RED for inconsistent, YELLOW for uncertain, and GREEN for consistent. A real-time consistency is also displayed

Dynamics Metrics
----------------

.. image:: ../data/cp_dynamics_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's dynamics diagnostic metrics.

|

The Dynamics Metrics section shows the following metrics if configured:

- **Input Domain signed distance**: This let you visualize the signed distance of the input domain to it's boundaries.

- **State Domain signed distance**: This let you visualize the signed distance derivative of the state domain to it's boundaries.

- **Degradation probability**: This let you visualize the probability of the system being in a degraded state based on the distance between computed and measured state evolution.

- **Simulated vs measured state difference**: This let you visualize the difference between the simulated and measured state at during the last integration period.

- **Dynamic Consistency**: This let you visualize the consistency score of the dynamics model with the measured state in the last integration period.
