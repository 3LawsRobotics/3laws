Diagnostics Page
================

The Diagnostic page provides a summary of the diagnostic metrics computed by the Supervisor. Presented as plots or status timeline, these metrics are useful for debugging and tuning the Supervisor's parameters.
Divided into sections, the Diagnostic page shows the following metrics:

- System metrics
- Odometry metrics
- Dynamics metrics

System Health metrics
---------------------

.. image:: ../data/cp_system_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's system diagnostic metrics.

|

The System Metrics section shows the following metrics:

- **System health tree**
- **CPU load**
- **Memory usage**
- **Disk usage**
- **Network usage**
- **Process counts**

Control tracking health metrics
-------------------------------

.. image:: ../data/cp_diagnostic_control_tracking.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's system diagnostic metrics.

|

This tab show for each controller the tracking error as a norm and per component (if bounds are specified)

Odometry Metrics
----------------

.. image:: ../data/cp_odometry_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's odometry diagnostic metrics.

|

The Odometry Metrics section shows the following metrics if configured:

- **Odometry General**: This let you visualize a component of the odometry across all available ROS odometry topics.
- **Diagnostic**: This let you visualize a timeline of the status of each odometry. The timeline can have 3 colors RED for inconsistent, YELLOW for uncertain, and GREEN for consistent. A real-time consistency is also displayed

Dynamics Metrics
----------------

.. image:: ../data/cp_dynamics_diagnostic.png
  :align: center
  :width: 800px
  :alt: Diagnostic page showing the Supervisor's dynamics diagnostic metrics.

|

The Dynamics Metrics section shows the following metrics if configured:

- **Input Domain signed distance**: This let you visualize the signed distance of the input domain to it's boundaries.

- **State Domain signed distance**: This let you visualize the signed distance derivative of the state domain to it's boundaries.

- **Degradation probability**: This let you visualize the probability of the system being in a degraded state based on the distance between computed and measured state evolution.

- **Simulated vs measured state difference**: This let you visualize the difference between the simulated and measured state at during the last integration period.

- **Dynamic Consistency**: This let you visualize the consistency score of the dynamics model with the measured state in the last integration period.
