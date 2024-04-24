.. _supervisor activation:

Supervisor
==========

Configuration
*************

The Configuration > Supervisor page contains configuration entries for both the monitoring and Copilot components.

- **Upload log to 3laws robotics cloud**: For debugging purposes, 3Laws creates a log file when Supervisor is started. The file is stored in *~/.3laws/logs*. Enabling this option allows 3Laws to provide better support with troubleshooting if there is a problem.

  * **World Frame**: Similar to "base robot frame", the name of the world frame (typically *odom* or *map*) must be specified.

  * **Advanced Settings > Project to SE2**: By default, the system is assumed to operate in 3-dimensional space. Projecting to SE2 assumes that the vehicle is traveling on a flat surface or that its travel distance is small enough that earth curvature effects are not significant.  When mapping from 3-dimensional space to 2-dimensional space, the system is assumed to have zero roll and pitch while being placed on the ground. No rotational velocities or acceleration are assumed around roll and pitch, and zero vertical velocity and acceleration are assumed.

  * **Advanced Settings > Process niceness**:  The computational priority of the Supervisor node can be set through the "niceness" parameter, where -20 would set it as very high priority and +19 would be very low priority. A niceness of zero is recommended. (Please review documentation on setting priority in Linux using "nice" for a deeper explanation.)

  * **Advanced Settings > Retimestamp policy** is used to add or correct the timestamp on log messages that seem to have an incorrect one. Leaving the timestamp unchanged is also an option.

* **Copilot**: *Input Redirection* The Copilot enables the run-time assurance capability where desired commands to the robot from the autonomy stack ("desired inputs") are modified in order to avoid collisions, and altered versions are published through a separate message.

  * **Desired control input**: This is the set of commands requesting speed and rotation (or speed and steering) that the autonomy stack is publishing. The ROS message type is needed so that Supervisor knows what to monitor in order to calculate the barrier function value. The message quality and receipt rate are monitored as part of the aggregated metrics, and if it fails to arrive within the expected time [1/(signal rate) * Timeout factor], an event will be created and the Copilot will transition to the failure command mode.

  * **Computed safe control input**: The right side of this area is purely informational. However, if the robot is to be controlled by the run-time assurance signal, it needs to subscribe to the message that is presented here. Alternatively, the launch file for Supervisor can be modified so that */lll/ram/filtered_input* is remapped to the command signal expected by the platform.

  * **Parameters > Activate**: This checkbox controls whether the run-time assurance intercepts and modifies commands from the planner/trajectory generator and forwards modified versions to the vehicle. The Copilot will only modify the outputs if the option is activated. If it is not activated, the unmodified "desired control input" will be transmitted on the designated "Computed safe control" message.  Additionally, when activated the CoPilot passes the unmodified desired input through to the platform except when a corrective action is needed.

  * **Parameters > Collision distance threshold**:  This is one of the most important values to set. This defines the distance between the edge of the robot and the nearest scan at which safety exists. If the measured distance drops below this value, the system is considered to be in an "unsafe" configuration.

  * **Parameters > Filter rate (hz)**: The frequency at which the run-time assurance publishes outputs. It is recommended that the run-time assurance run at the same rate as the desired control input or at a faster rate.

  * **Tuning > Aggressiveness**: This parameter controls how far from the nearest obstacle the safety filter starts having more effect on the commands and how strongly the safety filter pushes the robot back into the "safe" region if the safety definition has been violated. A larger value means that the control inputs from the planner will start to be modified when the robot is closer to an object/obstacle. That is, a larger value allows the platform to approach objects faster, and get closer.  If it gets too close, the Supervisor will push away from the object harder with a higher value.  A lower value will lead to a more tentative travel that stays farther away from objects.  In general lower values will produce larger margins. Typical values are between 0.5 and 1.0, but values in the range of 1000 might be used in reasonable situations.

  * **Tuning > Pointiness**: A rectangular (box) shape has an unintended behavior that if the robot comes towards a narrow object diretly in front of it, the closest point calculation will select the center-line of the box as the evaluation point for distance.  If the outer corners get closer to the object, the barrier function will want to increase the distance between the object and the box, so it will tend to center the box relative to the object.  A more desirable behavior is for the box to turn away from the object to be able to get around it.  The *pointiness* parameter is a way of accomplishing this.  If the box has a nose with sharper curvature, the barrier function will push it away from the object laterally, rather than simply trying to increase the measured closest distance between the box and the object.  So increasing the "pointiness" will encourage the vehicle to "turn away" from obstacles that are directly in front.

  * **Tuning > Avoidance Behavior**: Similar to pointiness, the alternate behavior can also be selected as choosing to slow down more or choosing to turn away from the object more as the commanded (desired) input drives the platform towards an object.

  * **Fault Management > Failure Command Mode**: The run-time assurance constantly monitors to ensure that it has enough data to determine whether the robot is in a safe condition. The minimum data required is the vehicle state, the laser scan values, and the commanded/desired input. If any of these is missing the RTA can switch to the failure command mode:

   * **Send Zero**:  In this mode the run-time assurance commands zero speed and zero turn/rotation in order to bring the vehicle to a stop.

   * **Do not Publish**:  Another option is to stop publishing values. This option should only be used if the robot has its own mechanism to put itself in a safe condition if it is not receiving commands.

  * **Fault Management > Can resume from failure**: With this checkbox filled in, once the input data (control input, laser scan, and state) values start appearing after a failure, the robot will be commanded back into motion (if the desired control input is asking for that). If the box is unchecked once there is a failure, the robot will remain stopped until the Supervisor is restarted.

  * **Advanced Settings > Accept wrong size laserscan**: One of the checks that is made on the incoming data is that the laserscan is delivering the expected number of scan points each frame. However, there are many laser scanners that are not consistent in the number of scan points they deliver. Checking this option allows for laser scanners with non-constant number of scan points reported.

  * **Advanced Settings > Use localization**:  Supervisor provides a MarkerArray that displays the robot's bounding box and rays to the closest obstacles. If "Use Localization" is set, the display is created relative to the world frame. In situations where the localization may be less reliable, this checkbox can be deselected, and the visualization will be based on the current robot base frame. Localization is also very useful if the control rate is low (e.g. longer times between commands) or there are delays between sensing and actuating. If the robot's motion is large during the time period of the control calculation, the model will account for it as long as localization is accurate.

- **Monitor**:

  Supervisor can publish a variety of diagnostic messages related to the health of the system clock, the dyamic consistency of the motion of the platform, individual node health, signal coherency, and summarized system health.  The published messages are discussed in :ref:`published_topics`.   These messages in the */lll/rdm* domain are only published if the Monitor is set to Active.

   * **Activate**: Enable publication of the diagnostic messages through the */lll/rdm* domain.  Faults detected in these variables do not cause CoPilot to switch to the failsafe mode.

   * **Timeout Factor**: Allows this many messages at the expected arrival rate to be missed before reporting an error.

   * **Maximum Delay (s)**: Maximum amount of time that a message can fail to appear before reporting an error.

The bottom section relates to republishing the control commands to the robot that are being sent from the autonomy stack. The values will be published on the *lll/ram/filtered_input* channel if the Copilot is activate or not. However, the values will only be different from the *Desired control input* if the Copilot is active.

- **Supervisor activation logic**:

  * **Finite States** are messages that the Diagnostics can listen to and issue events when the value of the finite state matches a predefined value.


.. _published_topics:

Topics published
****************

Runtime Assurance Module
------------------------

The Supervisor publishes a number of messages that can be used to monitor how the Run-time assurance behaves.

These messages are published on ROS topics and can be used by other ROS nodes. The ROS namespace for these topics is ``/lll``.

The following topics are published by the Supervisor by default:

- ``/lll/supervisor/ping``: A message that is published once per second to indicate that the Supervisor is running. This message contains statistics of the cpu usage, RAM usage, thread count, input/output, network traffic, and logging as used/produced by the Supervisor.

When the CoPilot/Run-time assurance capability is enabled, the following additional signals are published:

- ``/lll/ram/filtered_input``: The filtered velocity command that is sent to the robot's actuators when the Supervisor receives inputs from the planner/upstream.  Note that this signal may be remapped to match the name of the input the downstream systems expects.
- ``/lll/ram/markers``: Visualization tools such as RVIZ and FoxGlove can subscribe to this signal in order to display the illustrated vectors between the robot and obstacles that the Supervisor uses to make decisions on modifications to the input signal.  The image below shows an rviz2 display where the red box represents the robot, the blue vectors represent the vector between a sensed laser point and the closest point on the robot's boundary, and the green dots represent laser scan points that are being actively monitored by Supervisor. These values are all embedded into the ``/lll/ram/metadata``.  The white dots are the 2D laser scan points that are subscribed-to separately.

.. figure:: ../../data/rviz2.png
  :width: 600px
  :align: center
  :alt: Architecture schema

|

- ``/lll/ram/enable`` is a ROS Bool value that can be use to **command** the activation of the Supervisor.  The Copilot portion of the Supervisor needs to be Activated in order to use this signal.  Setting it to False will deactivate the Copilot filter.

- ``/lll/ram/metadata`` contains significant statistics about the monitor and the filter, particularly notifications of sensed delays, non-finite values, timeouts, data size variations, and initialization for inputs, states, and perception.  It also reports the most recent detected fault, along with several other useful details.


Robot Diagnostic Module
------------------------

The following topics are published by the Supervisor's diagnostics:


- ``/lll/rdm/clock_health_utc``: Statistics reporting variations in the synchronization of the robot clock and Universal time. This presents the current UTC time and real-time-clock value in nanoseconds since 1970, presenting the offset between the two.
- ``/lll/rdm/domain_status``: The summary of the condition of the vehicle is presented in the following fields: system_status, behavior_status, hardware_status, perception_status, and control_status.  When everything is working properly, a status of OK is reported for each.
- ``/lll/rdm/dynamic_consistency``: If a dynamical model has been added to the configuration, this topic will compare the predicted model behavior to the current behavior and provide discrepancy metrics.  Several values including differences in speeds between the platform and the model, and statistics on these differences are provided.  Time since last input and time since last state message are also provided (in nanoseconds).
- ``/lll/rdm/incidents_log``: As events are detected in the system, they are published through this channel as text messages.  The "tags" area of the message includes discrete-value key-value pairs for severity and type of issue.
- ``/lll/rdm/node_health``: Node health provides the status messages from various nodes. For example, the sensor_node provides details of the laserscan including the time since the last message, and average/min/max sending rates, receipt rates, and delays.
- ``/lll/rdm/sensor_noise``: This metric presents statistics on the noise characteristics for the sensors including: average error, maximum error, maximum angle error, and percentage of sigma.  Each data set is tagged with the sensor it corresponds to.
- ``/lll/rdm/sensor_obstruction``: If the monitor detects that the laserscan appears to be obstructed in certain angular sectors, the information about the amount of osbtruction is published through this channel.
- ``/lll/rdm/signal_health``: Monitor is constantly checking to see if inputs to the system are reasonable (e.g. are finite, numerical values). If values such as Not-a-Number (nan), purely zero, or infinity are received, the occurrences are reported.
- ``/lll/rdm/systems_health``: Detailed information about the system resource usage is provided through this channel: system_id, cpu_load, ram_usage, disk_usage, network_read, network_write, cpu_nb (core count), and procs_nb (process count).

More precise control can be obtained by editing the *supervisor.yaml* file under the *robot_diagnostics_module* area. The default settings are shown below.

.. literalinclude:: rdm_data.txt
