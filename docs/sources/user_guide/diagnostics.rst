Robot Diagnostic
################


The supervisor has a diagnostic component that monitor signals and compute relevant data for the robot safety.

This signals metadata and computed metrics are then published on ros in different topics so that the robot can use them to take decisions.

For a detail list of published ros topics, please refer to the :doc:`ros_interface` page.

The diagnostic component can only compute data from signals that are describe in the configuration. As for now, only signal used by the Collision avoidance modules are configurable.

.. note::
    The diagnostic component is not yet fully implemented in the supervisor. Further versions will increase its capabilities and configuration.
