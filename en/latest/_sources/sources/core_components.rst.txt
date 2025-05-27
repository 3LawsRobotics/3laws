Core components
###############

.. toctree::
  :maxdepth: 2
  :hidden:

  Run-time Assurance <core_components/runtime_assurance>
  Robot Diagnostics <core_components/diagnostics>
  CLI <core_components/cli>
  Reference <core_components/reference>


The Supervisor is delivered as a single package containing various tools and modules working together toward enabling these **sense-and-avoid capabilities**:

.. image:: data/architecture_detailed.png
  :align: center
  :width: 800px
  :alt: Detailed architecture schema

The Supervisor's core functionality is the :doc:`Runtime Assurance Module<core_components/runtime_assurance>` (RAM). This module is responsible for continuously filtering the desired commands sent by the planner to the vehicle in order to prevent unsafe behaviors in a minimally invasive way. This module is also responsible for implementing the fault management strategy in case a failure of a critical component of the system is detected.

Complementary to the RAM, the Supervisor integrates a :doc:`Robot Diagnostic Module<core_components/diagnostics>` (RDM). This module is responsible for monitoring the health and safety of these critical sub-systems more holistically. The monitoring results are available as metrics that are published in real-time on ROS topics. These metrics can be used to better understand the behavior of the robot and to trigger alternative actions.

In order for the RAM and RDM to work effectively, they need to be configured based on the robot's characteristics. This is done through the :doc:`Control Panel<first_time_use>` (CP), a web-based application that guides the user through the Supervisor configuration. CP also provides a way to visualize the robot's safety metrics in real time.

The Supervisor package also includes a :doc:`Command Line Interface<core_components/cli>` (CLI) for managing the Control Panel execution and Supervisor software updates from a terminal.

.. important::

  At launch time, the Supervisor node loads configuration from the YAML file located at ``~/.3laws/config/supervisor.yaml``. This file is created and updated by the CP. It can be manipulated manually for advanced configuration and backup.

.. important::

  The Supervisor generates a single log file located at ``~/.3laws/log/supervisor.log``. This file is overwritten at each launch. The logs are useful for debugging and monitoring the Supervisor's behavior. If you want to disable this file logging, you can specify an empty ``log_filepath`` :doc:`ROS parameter <core_components/reference/ros_parameters>` as part of the launch.
