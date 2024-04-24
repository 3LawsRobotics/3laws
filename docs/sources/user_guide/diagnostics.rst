Robot Diagnostic
################

.. warning::
  The Robot Diagnostic Module is currently in **experimental** phase. It is not yet fully implemented and its capabilities are limited.

The supervisor has a diagnostic component that monitor signals and compute relevant data for the robot safety.

This signals metadata and computed metrics are then published on ros in different topics so that the robot can use them to take decisions.

For a detail list of published ros topics, please refer to the :doc:`ros_interface` page.

The diagnostic component can only compute data from signals that are describe in the configuration. As for now, only signal directly used by the Run-time Assurance Module are configurable.
