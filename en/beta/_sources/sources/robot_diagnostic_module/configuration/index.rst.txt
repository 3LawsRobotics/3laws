Configuration
===============

.. contents:: Table of Contents
   :depth: 2
   :local:

The configuration is written inside a `YAML <https://yaml.org/>`_. file. This file can be stored anywhere but the default folder is
``/opt/3lawsRoboticsInc/robot_diagnostic_module/config``

Other path can be specified in the command line launch command

The **Robot Diagnostic Module** is shipped with a configuration template that can be found here
 ``/opt/3lawsRoboticsInc/robot_diagnostic_module/config/template_config.yaml``

In order to have the more user friendly configuration experience, a lot of the fields are optional.
The configuration can be build alongside your robot so that it does not represent a burden for the developer.


General configuration rules
-------------------------------

Bounds
^^^^^^^

All bounds parameters are similar:

- norm_type: Norm to use for norm bound, possible values (none, L_1, L_2, L_infinity)
- norm_upper_bound: Upper norm bound, floating point
- norm_lower_bound: Lower norm bound, floating point
- upper_bounds:  Upper bounds on the value of the components of the tracking_error, sequence of floating point
- lower_bounds: Lower bounds on the value of the components of the tracking_error, sequence of floating point
- rates_upper_bounds: Upper bounds on the value of the components of the tracking_error, sequence of floating point
- rates_lower_bounds:  Lower bounds on the value of the components of the tracking_error, sequence of floating point

Set both (rate_)upper_bounds[i] and (rates_)lower_bounds[i] to .inf to disable bound on component i.

Leave empty to deactivate all (rate) bounds checking.

Both (rate_)upper_bounds and (rates_)lower_bounds must have same size

Duration parameters
^^^^^^^^^^^^^^^^^^^^
Parameters like signal_min_rate represent durations.
They can be specified by an integer number corresponding to a number of nanoseconds:

  signal_min_rate: 1 -> 1 nanoseconds

  signal_min_rate: 1000000000 -> 1 second

They can also be specified by an integer of floating point followed by the time scale:

  signal_min_rate: 1ns -> 1 nanoseconds

  signal_min_rate: 25us -> 25 microseconds

  signal_min_rate: 0.5ms -> 500 milliseconds

  signal_min_rate: 1s -> 1 second

  signal_min_rate: 20hz -> 50 milliseconds

  signal_min_rate: 1min -> 1 minute

  signal_min_rate: 1h -> 1 hour

  signal_min_rate: 1d -> 1 day

  signal_min_rate: 1y -> 1 year

Scheme
-------

.. toctree::
   :maxdepth: 2
   :hidden:

   credentials/credentials
   diagnostic_config/diagnostic_config
   interface/interface
   robot_description/robot_description

The configuration has **4** distinct parts

- :doc:`credentials/credentials`:

  This part is here to specify the credentials to access the databases.
  These credentials are provided to you by email once your 3LawsRobotics account is created
- :doc:`diagnostic_config/diagnostic_config`:

  This part is the more important one. It describe your robot desired behavior,
  the interface data stream access for the RDM to monitor.
- :doc:`interface/interface`:

  Here are specified the diagnostic specific options such has the criticality of a certain event or the threshold above which a incident is flagged
- :doc:`robot_description/robot_description`:

  And this last part allow the user to specify interface specific options such has ROS Quality of Service

