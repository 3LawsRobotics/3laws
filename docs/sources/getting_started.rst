Getting Started
===============

This guide will help you install and configure the 3Laws Supervisor on your robot.

.. contents:: Table of Contents
   :depth: 2
   :local:


1. Installation
---------------

3Laws Supervisor is designed for installation on Ubuntu systems with ROS1 or
ROS2 already deployed on the computer.
To start the installation of Supervisor on your system, open a terminal and run the following command:

.. code-block:: bash

  bash <(curl https://raw.githubusercontent.com/3LawsRobotics/3laws/master/rdm/install.sh)

This script will download a package from github and will begin the installation. As it proceeds it will try to determine your system's configuration so that
the appropriate components are installed. The script will prompt you
for help with configuration if it is unable to find your computer's ROS distribution or architecture.

The script will add components to the global ROS installation.  These new
components will not be available until the ROS paths are updated. If your configuration automatically runs the ROS setup script when a new shell is started, please close the terminal and open a new one.  Otherwise, please run the following command:

.. code-block:: bash

  source /opt/ros/<DISTRO>/setup.sh

The Supervisor will now be available for operation.

2. Configuration
----------------

The 3Laws Supervisor is able to support several different types of robots and it
needs to connect to your system's data sources and sinks.  The configuration process aims to specify this type of information.

A graphical (browser-based) configuration tool (called **Control Panel**) is
available to help with the configuration effort.

Open a browser window and navigate to the address `http://localhost:8000`.
If this window does not appear, please manually start the Control Panel server
with the command:

.. code-block:: bash

  sudo systemctl start lll_control_panel

The Control Panel's capabilities can be augmented through a rosbridge server.
With that server, the control panel can more easily display some real-time
information about the Supervisor's status.  To install and start a rosbridge
server (where <rosdistro> is replaced with the version of ROS on your system):

.. code-block:: bash

  sudo apt-get install ros-<rosdistro>-rosbridge-server
  ros2 run rosbridge_server rosbridge_websocket

This will provide a websocket server at `ws://localhost:9090` that the control panel can connect to in order to retrieve topics and services information.


The first configuration page ("Setup") is the following:

.. image:: data/cpanel1.png
   :width: 800px
   :alt: Initial Control Panel page presenting Credentials, Robot name, and Company name

The 3Laws Supervisor is designed to work in conjunction with a cloud-based server.  When you purchase the Supervisor, 3Laws will provide you with the
credentials to connect to the web-based server and with the official company
name that is used for your serivces.

- **Credentials**: 3Laws will provide you with an Influx Key and a ClickHouse key so that your robot can connect to the cloud-based server and can send its summarized information for display through Grafana.
- **Robot Name**: This identifier will be different for each robot where you install Supervisor.  It should contain a name you can use to identify an individual robot.
- **Company Name**: As part of the registration process, your company's name will be used to create an account on the cloud server. Please make sure that the name matches what 3Laws used to create your account.  This name is used for logging into the cloud server.

The right half of this panel contains the Rosbridge connection status (upper right), configuration for the Rosbridge (gear next to the status), and a link to this documentation.  The "Update Instructions" section is purely informational.

The **Save** button on each page of the Control Panel should be pressed before moving on to another page.

The next section of the Control Panel is the "Configuration" page, which consists of sections (tabs) listed as *Robot Model*, *Supervisor*, *Localization*, and *Perception*.

Robot Model
-----------
The Configuration page for the Robot Model is where the robot's geometry and kinematics are specified.

.. image:: data/cpanel2.png
   :width: 800px
   :alt: Configuration > Robot Model page where the kinematics and geometry are specified. 

Throughout this documentation, a red asterisk (<span style="color:red">*</span>) indicates a *required* field.
  
- **Robot Type**: Supervisor currently supports differential drive, front-steered, and omni-directional mobile platforms.  A differential drive vehicle can rotate around a fixed location by driving one wheel forward and its pair in reverse. It can move forward or backwards by driving both wheels in the same direction.  A front-steered vehicle operates like a car or bicycle. It can move forwards or backwards, but direction is controlled by steering, and the vehicle has a fixed wheelbase distance between the forward axle and the rear axle. An omni-directional robot is usually implemented using wheels that have a series of smaller wheels mounted at 45 degrees on the outer rim.  By moving one axle forward and others backwards, the vehicle can move sideways. Rotation can be effected by moving the wheels on one side forwards and the wheels on the other side in reverse. 
- **Kinematics**: The vehicle's geometry and the name of the base frame (e.g. base_link, chassis, body) are specified in this section.

  * Frames: The name of the robot base frame must be specified here. Additional static frames can be defined by clicking the + icon. The additional frames are static, and will follow the motion of the base robot frame.

  * Shape: The robot's shape is used in order to calcuate the distance between the outer boundaries of the robot body and any scan points.  Basic shapes that are curently supported include sphere, box, capsule, point, cone and cylinder.  The parameter entries change based on the selected shape. Cylinders and capsules both require length and radius. The difference is that a capsule will have hemispheres on the ends while the cylinder ends are flat.

   * Shape Pose in Robot Frame: The relative orientation of the robot's shape with respect to the base frame needs to be specified. Note that the rotation can be specified either using quaternions or Euler angles. The quaternion order is w, x, y, z.

- **Dynamics**: The Dynamics section has 3 subsections: Input, Parameters, and State.
  
  * The Input section supports specification of maximum and minimum limits for the translational and rotational speeds at which the robot can be commanded. For monitoring, if these values are exceeded, an event to this effect will be issued. If the RTA component is active, these limits will be applied to the filtered outpus. For steered vehicles, instead of limits on rotational speeds, steering limits are specified.

  * Parameters: The only robot type that has inputs in the parameters section is the steered robot. *wheel_dx* is the wheelbase length for the vehicle. *origin_x* is the distance from the rear axle to the robot base frame.

  * State: For all robot types, the State category provides the definitions of the variables that are considered as the "states".  These definitions are important when trying to create "masks" to map between the input ROS variable types and the robot states.  The first, is considered state 0 (typically x position), the second is state 1 (typically y position) and the third is state 2 (typically yaw).

Remeber to save each page after inputing the data.

Supervisor
----------

The Configuration > Supervisor page contains configuraiton entries for both the RDM (monitor) and RTA (Copilot) components.

.. image:: data/cpanel3.png
   :width: 800px
   :alt: Configuration > Supervisor page where monitoring thresholds and run-time assurance settings are available. 


- **Basic configuration monitor**: There is a checkbox that needs to be select if there is a desire to upload statistics to the cloud database.

  * World Frame: Similar to "base robot frame", the name of the world frame (typically *odom* or *map*) must be specified.
  * Advanced Settings: Max delay (s) and Timeout Factor are thresholds for triggering events informing that data failed to arrive (if data is not received for max-delay * timeout-factor seconds.  If the copilot (run-time assurance) is active, failure to receive robot state or desired control input (in timeout factor * 1/signal-rate) will cause the copilot to switch to the Failure Command Mode (which is explained below).
  * Copilot
    
    * Activate: This checkbox controls whether the run-time assurance intercepts and modifies commands from the planner/trajectory generator and forwards modified versions to the vehicle.
      
    * Aggressiveness: This parameter controls when the safety filter starts having more effect on the commands and how strongly the safety filter pushes the robot back into the "safe" region if the safety definition has been violated.  A larger value means that the control inputs from the planner will start to be modified when the robot is farther from an object/obstacle.  In general this will produce larger margins.  A larger value also means that if an obstacle is detected within the collision distance, the command modified by the run-time assurance will try to move the robot away from the object more aggressively.  Typical values are between 0.5 and 1.0.

 The following are under the "Advanced Settings":

     * Failure Command Mode: The run-time assurance constantly monitor to ensure that it has enough data to determine whether the robot is in a safe condition. The minimum data required is the vehicle state, the laser scan values, and the commanded/desired input.  If any of these is missing the RTA can switch to the failure command mode:

      * Send Zero:  In this mode the run-time assurance commands zero speed and zero turn/rotation in order to bring the vehicle to a stop.

      * Do not Publish:  Another option is to stop publishing values.  This option should only be used if the robot has its own mechanism to put itself in a safe condition if it is not receiving commands.

      * Send desired: If there is a data dropout for the state or the laser scan, the RTA can instead just forward the control input that it is filtering without any modifications for safety.
  
      * Send Failsafe: This option is for future expansion - do not use at this time.

    * Yield on failure:  This checkbox is like the "Send Desired" option.  Setting this checkbox will override the failure control mode and just forward the input.
    * Can resume from failure: With this checkbox filled in, once the input data (control input, laser scan, and state) values start appearing after a failure, the robot will be commanded back into motion (if the desired control input is asking for that).  If the box is unchecked once there is a failure, the robot will remain stopped until the Supervisor is restarted.

    * Use localization:  Supervisor provides a MarkerArray that displays the robot's bounding box and rays to the closest obstacles.  If "Use Localization" is set, the display is created relative to the world frame.  In situations where the localization may be less reliable, this checkbox can be de-selected, and the visualization will be based on the current robot base frame.

    * Accept wrong size laserscan: One of the checks that is made on the incoming data is that the laserscan is delivering the expected number of scan points each frame. However, there are many laser scanners that are not consistent in the number of scan points they deliver.  Checking this option allows for laser scanners with non-constant number of scan points reported.

    * **Collision distance threshold**:  This is one of the most important values to set. This defines the distance between the edge of the robot and the nearest scan at which safety exists.  If the measured distance drops below this value, the system is considered to be in an "unsafe" configuration.

    * Filter rate (hz): This is the frequency at which the run-time assurance runs.  It is recommended that the run-time assurance run at the same rate as the desired control input or at a faster rate.

    * Conservativeness: This is a factor that specifies how much uncertainty the robot operator thinks there is in the localization and sensor data.  The ratio between aggressiveness and conservativeness is the main controller of the behavior.  Values below 0.1 are recommended.
      
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



