Getting started
################

.. contents:: Table of Contents
  :local:

The current version of **3Laws Supervisor** supports ground-based mobile platforms (wheeled or legged) with the following movement modalities:

 * Differential-drive
 * Omni-directional
 * Front-wheel steering - **experimental**

For perception, **3Laws Supervisor** currently supports:

 * Raw 2D laserscan data
 * Pre-processed map of obstacles with their locations and shapes

The **3Laws Supervisor** software is a `ROS <http://www.ros.org>`_ node available on x86_64 and ARM-64 processor architectures for multiple OS/ROS combinations:

+-----------------------+--------------+---------------------+
| Ubuntu Distribution   | ROS1 version |    ROS2 version     |
+=======================+==============+=====================+
|        22.04          |     N/A      |     Humble/Iron     |
+-----------------------+--------------+---------------------+
|        20.04          |     Noetic   |     Galactic/Foxy   |
+-----------------------+--------------+---------------------+

.. _Installation:

1. Install Supervisor
*********************

To install Supervisor on the system, open a terminal (on the target device) and run the following command:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/install.sh)

This will run a script to auto-detect the system architecture, install any missing dependencies, download the right binary, and guide you through any necessary configuration steps.

.. important::

  *sudo* permission may be required in order to run APT during software installation.

.. note::

  The ROS packages are installed into the global ROS installation directory. You will need to source the ROS setup script to make the new components available in the current terminal: ``source /opt/ros/<DISTRO>/setup.sh``.

  As with most ROS setups, adding a line to the startup file (ex. .bashrc) that sources the ROS environment is recommended.


2. Start the Control Panel
**************************
Before the Supervisor can be started, it must be configured. In order to configure it, a web-based :doc:`Control Panel <user_guide/control_panel>` is provided.  The Control Panel creates if needed and modifies the file *~/.3laws/config/supervisor.yaml*.

An existing (or backup) version of this file can be used if it is placed in the proper location; however, older versions might no be compatible with new software.  If this file is copied from another device, please update the license.

To enable the Control Panel backend service, open a terminal and run the following command:

.. code-block:: bash

  3laws control-panel autostart enable

.. note::
  See :doc:`CLI reference <user_guide/cli>` for more options to start the control panel.

3. Configure Supervisor
************************

Now that the Control Panel backend is running, access the control panel from any machine on the same network as the robot by opening a web browser and navigating to the following URL: ``http://<IP_ADDRESS_OF_THE_ROBOT>:8000/``.

The initial view of the Control Panel is the "Configuration" page, which consists of sections (tabs) listed as:

.. toctree::
  :maxdepth: 1

  1. Credentials <user_guide/configuration/credentials>
  2. Model <user_guide/configuration/robot_model>
  3. Supervisor <user_guide/configuration/supervisor>
  4. Localization <user_guide/configuration/localization>
  5. Perception <user_guide/configuration/perception>

.. warning::

  The entire configuration process needs to be completed before starting the Supervisor software. If a part of the configuration is missing, the associated tab will be orange in color. Once the configuration is complete all tabs should be white.

.. note::

  See :ref:`Control Panel reference <control_panel_config>` for more details on the configuration options.


.. note::

  The Supervisor does not have to run during the configuration step. It loads the configuration file at start-up, so it needs to be started **after** the configuration is created/updated. However, if the rest of the robot is running while the Supervisor is being configured, the Control Panel will pre-filled the configuration with available information.


4. Interface with your stack
*****************************

In order to perform collision avoidance maneuvers, the Supervisor must be able to send commands to your robot actuators. These commands will be published on the ``/lll/ram/filtered_input`` topic.

Your low-level controller therefore needs to subscribe to this topic and apply the commands to your robot:

.. image:: data/ram_interfacing.png
  :align: center
  :width: 600px
  :alt: Operations page showing a configured robot that does not yet have sensor or planning data.

For that, you can either create an extra dedicated subscriber in your low-level control stack to receive the commands from the Supervisor, or you can use ROS topic remapping feature to remap the low-level controller subscription from ``/cmd`` to ``/lll/ram/filtered_input``:

.. tabs::
  .. tab:: ROS1

    When running the control node:

    .. code-block:: bash

      rosrun controller controller /cmd:=/lll/ram/filtered_input

    Inside the control node launch file:

    .. code-block:: xml

      <launch>
        <remap from="/cmd" to="/lll/ram/filtered_input" />
        <node name="controller" pkg="controller" type="controller" />
      </launch>

    When including your low-level control stack launch file into another launch file:

    .. code-block:: xml

      <launch>
        <group>
          <include file="$(find controller)/launch/controller.launch" />
          <remap from="/cmd" to="/lll/ram/filtered_input" />
        </group>
      </launch>

  .. tab:: ROS2

    When running the control node:

    .. code-block:: bash

      ros2 run controller controller --ros-args -r /cmd:=/lll/ram/filtered_input

    Inside the control node launch file:

    .. code-block:: python

      Node(
          package="controller",
          executable="controller",
          output="screen",
          remappings=[
              ("/cmd", "/lll/ram/filtered_input"),
          ],
      )

    When including your low-level control stack launch file into another launch file:

    .. code-block:: python

      from ament_index_python.packages import get_package_share_directory
      from launch import LaunchDescription
      from launch.actions import GroupAction, IncludeLaunchDescription
      from launch.launch_description_sources import PythonLaunchDescriptionSource
      from launch.substitutions import PathJoinSubstitution
      from launch_ros.actions import SetRemap


      def generate_launch_description():

          launchDesc = LaunchDescription()

          launchDesc.add_action(
              GroupAction(
                  [
                      SetRemap("/cmd", "/lll/ram/filtered_input"),
                      IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(
                              PathJoinSubstitution(
                                  [
                                      get_package_share_directory("controller"),
                                      "launch",
                                      "controller.py",
                                  ]
                              )
                          )
                      ),
                  ]
              )
          )

          return launchDesc

5. Launch
*********

To launch the Supervisor directly, use the following command:

.. tabs::
  .. tab:: ROS1
    .. code-block:: bash

      roslaunch lll_supervisor supervisor.launch

  .. tab:: ROS2
    .. code-block:: bash

      ros2 launch lll_supervisor supervisor.launch.py


To include the Supervisor as part of your launch file, use the following code snippets:

.. tabs::
  .. tab:: ROS1
    .. code-block:: xml

      <include file="$(find lll_supervisor)/launch/supervisor.launch">
        <arg name="log_level" value="info"/>
      </include>"

  .. tab:: ROS2
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

6. Monitor your system (optional)
*********************************

The Control Panel provides an `Operation` page that can be used to monitor the status of the Supervisor working with your stack.

The Operation page requires that both the Supervisor and the Control Panel backend are running. Additionally, a `rosbridge websocket <https://github.com/RobotWebTools/rosbridge_suite>`_ must be running on the same network as the Supervisor.

To install the rosbridge suite, run the following command:

.. code-block:: bash

  sudo apt-get install ros-<DISTRO>-rosbridge-suite

To start the rosbridge websocket, run the following command:

.. tabs::
   .. tab:: ROS1
     .. code-block:: bash

       roslaunch rosbridge_server rosbridge_websocket.launch

   .. tab:: ROS2
     .. code-block:: bash

       ros2 launch rosbridge_server rosbridge_websocket_launch.xml

.. important::

  Make sure to specify the rosbridge websocket IP address and port in the Control Panel if using something other than the defaults:

  .. image:: data/cpanel7.png
   :align: center
   :width: 600px
   :alt: Operations page showing a configured robot that does not yet have sensor or planning data.

.. note::

  See :ref:`Control Panel reference <control_panel_ops>` for more details on the operation page.


7. Update (optional)
********************

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

Continue with :doc:`User Guide<user_guide>` to discover everything the Supervisor can do.
