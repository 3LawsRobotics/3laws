Getting started
################

.. contents:: Table of Contents
  :local:

The current version of **3Laws Supervisor** supports ground-based mobile platforms (wheeled or legged) with the following movement modalities:

 * Differential-drive
 * Omni-directional
 * Omni-directional mobile manipulator

For perception, **3Laws Supervisor** currently supports:

 * Raw 2D laserscan data
 * Raw 3D Point Cloud data
 * Pre-processed map of obstacles with their locations and shapes

The **3Laws Supervisor** software is a deployed as a `ROS <http://www.ros.org>`_ node available on x86_64 and ARM-64 processor architectures for multiple OS/ROS combinations:

+-----------------------+---------------------+
| Ubuntu Distribution   |    ROS2 version     |
+=======================+=====================+
|        24.04          |        Jazzy        |
+-----------------------+---------------------+
|        22.04          |     Humble/Iron     |
+-----------------------+---------------------+
|        20.04          |     Galactic/Foxy   |
+-----------------------+---------------------+

.. _Installation:

1. Install Supervisor
*********************

To install Supervisor on the system, open a terminal (on the target device) and run the following command:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh)

This will run a script to auto-detect the system architecture, install any missing dependencies, download the binaries, and guide you through any necessary configuration steps.

.. important::

  *sudo* permission may be required in order to run APT during software installation.

.. note::

  The ROS packages are installed into the global ROS installation directory. You will need to source the ROS setup script to make the new components available in the current terminal: ``source /opt/ros/<DISTRO>/setup.sh``.


2. Start the Control Panel
**************************
Before the Supervisor can be started, it must be configured. In order to configure it, a web-based :doc:`Control Panel <first_time_use>` is provided.  The Control Panel creates and modifies the file *~/.3laws/config/supervisor.yaml*.

An existing (or backup) version of this file can be used if it is placed in the proper location; however, older versions might no be compatible with new software.  If this file is copied from another device, the license key have to be changed.

To enable the Control Panel service, open a terminal and run the following command:

.. code-block:: bash

  3laws control-panel autostart enable

.. note::
  See :doc:`CLI reference <core_components/cli>` for more options to start the control panel.

3. Configure Supervisor
************************

Now that the Control Panel service is running, access the control panel from any machine on the same network as the robot by opening a web browser and navigating to the following URL: ``http://<IP_ADDRESS_OF_THE_ROBOT>:8000``.

The initial view of the Control Panel is the "Configuration" page, which consists of sections (tabs) listed as:
:doc:`General <configuration/general>`
, :doc:`Model <configuration/robot_model>`
, :doc:`Supervisor <configuration/supervisor>`
, :doc:`Localization <configuration/localization>`
, :doc:`Perception <configuration/perception>`

.. warning::

  The entire configuration process needs to be completed before starting the Supervisor software. If a part of the configuration is missing, the associated tab will be orange in color. Once the configuration is complete all tabs should be white.

.. note::

  See :ref:`Control Panel reference <control_panel_config>` for a detailed description and walk-through of the configuration options.


.. note::

  The Supervisor does not have to run during the configuration step. It loads the configuration file at start-up, so it needs to be started **after** the configuration is created/updated.

  **Optional**: In order to help during the supervisor configuration phase, it is possible to run a `rosbridge websocket <https://github.com/RobotWebTools/rosbridge_suite>`_ instance on the same network as the Control Panel. See :ref:`Rosbridge connection <rosbridge_connection>` for more details on how to configure a rosbridge websocket connection.

4. Interface with your stack
*****************************

In order to perform collision avoidance maneuvers, the Supervisor must be able to send commands to your robot actuators. These commands will be published by default on the ``/lll/ram/filtered_input`` topic *(Customizable name via the Control Panel)*.

Your low-level controller therefore needs to subscribe to this topic and apply the commands to your robot:

.. image:: data/supervisor_interfacing.png
  :align: center
  :width: 600px
  :alt: Operations page showing a configured robot that does not yet have sensor or planning data.

|

For that, you can either create an extra dedicated subscriber in your low-level control stack to receive the commands from the Supervisor, or you can use ROS topic remapping feature to remap the low-level controller subscription to accept messages from ``/lll/ram/filtered_input``.

5. Launch
*********

To launch the Supervisor directly, use the following command:

.. code-block:: bash

  ros2 launch lll_supervisor supervisor.launch.py


To include the Supervisor as part of your launch file, use the following code snippets:

.. code-block:: python

  from ament_index_python.packages import get_package_share_directory
  from launch import LaunchDescription
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from launch.substitutions import PathJoinSubstitution

  def generate_launch_description():

      launchDesc = LaunchDescription()

      launchDesc.add_action(
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                  PathJoinSubstitution(
                      [
                          get_package_share_directory("lll_supervisor"),
                          "launch",
                          "supervisor.launch.py",
                      ]
                  )
              ),
              launch_arguments={
                  "log_level": "info",
              }.items(),
          )
      )

      return launchDesc

If ROS is unable to find the ``lll_supervisor``, re-run the source command for the ROS paths.

6. Monitor your system
***********************

The Control Panel provides an `Operation` page that can be used to monitor the status of the Supervisor working with your stack and a `diagnostic` page to display metrics in realtime.

These pages require that both the Supervisor and the Control Panel backend are running.

.. note::

  See :ref:`Control Panel reference <control_panel_ops>` for more details on the operation page.


7. Update
*********

You can check for updates to the Supervisor by running the following command:

.. code-block:: bash

  3laws check-update

To update the Supervisor, use the same command as for the installation:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh)

.. note::

  The supervisor will be updated to the latest version available for the system's distribution. The **existing configurations will not be modified**, but if new variables need to be configured, advisories will be given during the installation.

8. What's next?
****************

Continue with :doc:`First Time Use<first_time_use>` for a step-by-step guide on configuring the supervisor or with :doc:`Core Components<core_components>` to discover everything the Supervisor can do.
