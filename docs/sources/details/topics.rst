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
