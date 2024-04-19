Supervisor ROS topics
*********************

The Supervisor publishes a number of messages that can be used to monitor how the Run-time assurance behaves.

These messages are published on ROS topics and can be used by other ROS nodes. The ROS namespace for these topics is ``/lll``.

The following topics are published by the Supervisor by default:

- ``/lll/supervisor/ping``: A message that is published once per second to indicate that the Supervisor is running. This message contains statistics of the cpu usage, RAM usage, thread count, input/output, network traffic, and logging as used/produced by the Supervisor.

When the CoPilot/Run-time assurance capability is enabled, the following additional signals are published:

- ``/lll/ram/filtered_input``: The filtered velocity command that is sent to the robot's actuators when the Supervisor receives inputs from the planner/upstream.  Note that this signal may be remapped to match the name of the input the downstream systems expects.
- ``/lll/ram/markers``: Visualization tools such as RVIZ and FoxGlove can subscribe to this signal in order to display the illustrated vectors between the robot and obstacles that the Supervisor uses to make decisions on modifications to the input signal.  The image below shows an rviz2 display where the red box represents the robot, the blue vectors represent the vector between a sensed laser point and the closest point on the robot's boundary, and the green dots represent laser scan points that are being actively monitored by Supervisor. These values are all embedded into the ``/lll/ram/metadata``.  The white dots are the 2D laser scan points that are subscribed-to separately.

.. image:: ../data/rviz2.png
   :width: 600px
   :alt: Architecture schema

- ``/lll/ram/enable`` is a ROS Bool value that can be use to **command** the activation of the Supervisor.  The Copilot portion of the Supervisor needs to be Activated in order to use this signal.  Setting it to False will deactivate the Copilot filter.

- ``/lll/ram/metadata`` contains significant statistics about the monitor and the filter, particularly notifications of sensed delays, non-finite values, timeouts, data size variations, and initialization for inputs, states, and perception.  It also reports the most recent detected fault, along with several other useful details.

The following topics are published by the Supervisor's diagnostics:

- ``/lll/rdm/clock_health``: Statistics reporting variations in the synchronization of the robot clock and Universal time.
- ``/lll/rdm/dynamic_consistency``: If a dynamical model has been added to the configuration, this topic will compare the predicted model behavior to the current behavior and provide discrepancy metrics.
- ``/lll/rdm/node_health``: Node health gives a general view of the health of the robot components likes localization, perception, inputs, and states.
- ``/lll/rdm/sensor_characterization``: This metric presents statistics on the noise characteristics, data dropouts, anomalies, and potential obstructions of the sensors.
- ``/lll/rdm/systems_health``: Detailed information about the system resource usage (CPU, RAM, etc.) is provided through this channel.
- ``/lll/rdm/signal_health``: All Supervisor input signals (state, commands, perception) are treated as signals with a specific frequency. This metric gives information about the signal health (timeout, delays, invalid data) based on the configured expected signal rates.

