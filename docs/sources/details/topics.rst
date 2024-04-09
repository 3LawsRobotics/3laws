Supervisor ROS topics
*********************

The Supervisor publishes a number of messages that can be used to monitor how the Run-time assurance behave.

These messages are published on ROS topics and can be used by other ROS nodes. The namespace for these topics is ``/lll``.

The following topics are published by the Supervisor by default:

- ``/lll/supervisor/ping``: A message that is published at a regular interval to indicate that the Supervisor is running.

And when the RTA capability is enabled:

- ``/lll/ram/filtered_input``: The filtered velocity command that is sent to the robot's actuators.
- ``/lll/ram/metadata``: Metadata about the supervisor filter, the robot's state and input.
- ``/lll/ram/enable``: A boolean to **command** the activation of the Supervisor.


The following topics are published by the Supervisor's diagnostics:

- ``/lll/rdm/clock_health``: A message about the sync of the robot clock and the universal time.
- ``/lll/rdm/dynamic_consistency``: If a dynamical model as been added to the configuration, this topic will compare the predicted model behavior to the current behavior and provide discrepancies metrics.
- ``/lll/rdm/node_health``: Node health gives a general view of the health of the robot components likes localization, perception, etc.
- ``/lll/rdm/sensor_characterization``: This metric gives details on the noise and the potential obstruction of the sensors.
- ``/lll/rdm/systems_health``: Give information about the system resources.
- ``/lll/rdm/signal_health``: All entry topic of the supervisor are treated as signals with a specific frequency. This metric gives information about the signal health (timeout, delays, invalid data).
