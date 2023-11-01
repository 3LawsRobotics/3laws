Configuration
===============

The configuration is written inside a `YAML <https://yaml.org/>`_. file. This file can be stored anywhere but the default folder is
``/opt/3lawsRoboticsInc/robot_diagnostic_module/config``

Other path can be specified in the command line launch command

The **Robot Diagnostic Module** is shipped with a configuration template that can be found here
 ``/opt/3lawsRoboticsInc/robot_diagnostic_module/config/template_config.yaml``

In order to have the more user friendly configuration experience, a lot of the fields are optional.
The configuration can be build alongside your robot so that it does not represent a burden for the developer.

Scheme
======

The configuration has **4** distinct parts

- :doc:`credentials/credentials` (Credentials):

  This part is here to specify the credentials to access the databases.
  These credentials are provided to you by email once your 3LawsRobotics account is created
- :doc:`diagnostic_config/diagnostic_config` (Diagnostic Configuration):

  This part is the more important one. It describe your robot desired behavior,
  the interface data stream access for the RDM to monitor.
- :doc:`interface/interface` (Interface):

  Here are specified the diagnostic specific options such has the criticality of a certain event or the threshold above which a incident is flagged
- :doc:`robot_description/robot_description` (Robot Description):

  And this last part allow the user to specify interface specific options such has ROS Quality of Service

.. toctree::
   :maxdepth: 2

    credentials/credentials
    diagnostic_config/diagnostic_config
    interface/interface
    robot_description/robot_description