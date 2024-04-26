Robot Diagnostic
################

.. warning::
  The Robot Diagnostic Module (RDM) is currently in **experimental** phase. 3Laws is actively working on expanding and maturing its capabilities.

.. contents:: Table of Contents
  :local:



The Supervisor's RDM is a tool for monitoring the health and safety of various critical sub-systems of the robot. It is designed to work in tandem with the Supervisor's Run-time Assurance Module (RAM).

As explained :doc:`here <runtime_assurance>`, the RAM relies on various critical robot sub-systems to perform it's collision avoidance and safety functions. The fault Management part of the RAM is able to detect obvious faults in these sub-systems, like timeouts or corrupted messages,
but it is not able to detect more subtle issues that may arise, like inconsistent localization, or faulty sensor. The RDM is designed to fill this gap by providing a more thorough and holistic monitoring of these critical components for ensuring collision free behavior.

.. warning::
  At the moment, the diagnostic performed by the RDM is not fed to the RAM's Fault Management system. It is up to the user to decide how to act on the information provided by the RDM.


Configuration
==============
In order for the RDM to perform its analysis, it must be fed data about the system. At the moment, the RDM is only fed the data directly required for the RAM and specified via the control panel. 3Laws is actively working on enabling more data sources to be configured via the Control Panel.


Metric Modules
==============

The RDM is composed of a set of **metric modules** each of which responsible for analyzing a specific aspect of the robot's health. Each module then streams a set of **metrics**.

.. important::
  Please refer to the :ref:`reference_ros_topics` page for the list of ROS topics these metrics are published on.

.. _contact 3Laws: mailto:sales@3laws.io

.. note::
  The RDM is designed to be modular and extensible. For more information on what other modules and metrics 3Laws can offer, please `Contact 3Laws`_.

The following modules are currently available:

Clock Health
------------

A properly functioning robot should have a consistent clocks across all its components. The Clock Health module monitors the synchronization between `system clock <https://en.cppreference.com/w/cpp/chrono/system_clock>`_ of the computer the Supervisor is running on and an `NTP server <https://www.ntppool.org/>`_. Inconsistent clocks can lead to a variety of issues, including inaccurate, improper timeout and communication detection by the RAM's Fault Management system.

This module publishes the following metrics:

- **clock_health_utc**: The offset between the system clock and the NTP server in seconds. The important fields of the associated message are:

  - **has_utc**: A boolean indicating if the NTP server time is available.

  - **utc_time**: The latest time returned by the NTP in nanoseconds since 1970.

  - **rtc_time**: The latest time returned by the System Clock in nanoseconds since 1970.

  - **offset_from_utc**: The offset between the NTP server and the System Clock in nanoseconds.


Dynamic Consistency
----------------------

The RAM (Run-time Assurance Module) relies on a quantitative understanding of the robot's physical behavior and control capabilities (i.e. a `Dynamical Model of the system <https://en.wikipedia.org/wiki/Dynamical_system>`_) to make decisions about the appropriate collision avoidance strategy. The Dynamic Consistency module is responsible for monitoring the consistency between the Dynamical Model of the system and the observed system behavior. Inconsistencies between the two implies the RAM may not be able to make accurate decisions on when to intervene and lead to collisions.

This module publishes the following metrics:

- **dynamic_consistency**: Consistency between the Dynamical Model of the system and the observed system behavior. The important fields of the associated message are:

  - **state_timeout**: A boolean indicating if the state information has timed out.

  - **input_timeout**: A boolean indicating if the input information has timed out.

  - **input_domain_value**: The signed distance from the current input to the boundary of the input constraints set.

  - **state_domain_value**: The signed distance from the current state to the boundary of the state domain set.

  - **xdot_difference**: The difference vector between the predicted and observed state derivative.

  - **xdot_difference_pdf_value**: The probability density function value of the xdot_difference vector.

  - **xdot_difference_pdf_value_normalized**: The normalized probability density function value of the xdot_difference vector, i.e. equal to 1 when the xdot_difference vector is null.

  - **xdot_difference_norm_1sigma**: The signed distance from the value of the norm of xdot_difference to the 1sigma level set of the pdf.

  - **xdot_difference_norm_2sigma**: The signed distance from the value of the norm of xdot_difference to the 2sigma level set of the pdf.

  - **xdot_difference_norm_3sigma**: The signed distance from the value of the norm of xdot_difference to the 3sigma level set of the pdf.

  - **system_degradation_probability**: Not available yet.

.. important::
  Currently, the process covariance matrix used for the statical analysis is the identity matrix. This will be configurable in future versions of the RDM.


Node Health
-----------

Typical autonomy stacks are composed of **multiple nodes**, each responsible for a specific task. The Node Health module monitors the health of these nodes by checking if they are running and if they are publishing data properly. This module is useful for detecting issues like node crashes or communication issues between nodes. This is particularly useful for nodes like Localization and Perception that are critical to the RAM's proper operation.

A node is defined by the set of topics it publishes. The Node Health module monitors the health of the nodes by checking if the topics are being published and if the data is consistent with the expected values.

The associated metric is published at 1hz, and is an aggregate of the signal data received over that period.

This module publishes the following metrics:

- **node_health**: The health of the nodes in the system. The important fields of the associated message are:

  - **timeout**: A boolean indicating if all the nodes's topics have timed out.

  - **ok**: A boolean indicating if any of the nodes's topics have timed out.

  - **error_code**: An enum indicating the type of error that occurred. The possible values are: **[ok, some_topics_timeout, out_of_bounds, all_topics_timeout]**

  - **topics**: A list of info for each of the node's topics. Each topic message contains the following important fields:

    - **topic_id**: The id of the topic.

    - **timeout**: A boolean indicating if the topic has timed out.

    - **has_timestamp**: A boolean indicating if the topic has a non-zero timestamp.

    - **sender_rate**: The average rate at which the topic is being published by the node.

    - **receiver_rate**: The average rate at which the topic is being received by the RDM.

    - **delay**: The average delay between the time the message was sent and the time it was received.


Sensor characterization
------------------------

The Sensor Characterization module is responsible for monitoring the health of the robot's sensors. This module is useful for detecting issues like sensor malfunctions, or calibration issues. The nominal functioning of the robot's sensors is obviously critical for the RAM's proper operation. If the sensors are not functioning properly, the RAM will make decision based on improper assumption on obstacles location and may lead to collisions.

This module publishes the following metrics:

- **sensor_obstruction**: The list of sensor obstructions detected for the specified 2D laserscans. An obstruction is detected when a set of rays returns the same distance over time even through the robot is moving. The important fields of the associated message are:

  - **nb_obstructions**: The number of obstructions detected.

  - **obstructions**: A list of the detected obstruction clusters. Each obstruction message contains the following important fields:

    - **start_angle**: The start angle of the obstruction in radians in the laserscan frame.

    - **end_angle**: The end angle of the obstruction in radians in the laserscan frame.

    - **min_dist**: The minimum distance of the rays points in that obstruction cluster in meters.

    - **max_dist**: The maximum distance of the rays points in that obstruction cluster in meters.

    - **type**: The type of the obstruction. The possible values are: **[near, middle, far]**. Near obstructions are detected when the rays are below the minimum range configured, far obstructions are detected when the rays are beyond the maximum range configured, and middle obstructions are detected in between.

- **sensor_noise**: This metric presents statistics on the noise characteristics for the sensors. The important fields of the associated message are:

  - **average_std_error**: The standard deviation of all the sensor measurements over 1s.

  - **max_std_error**: The maximum deviation of all the sensor measurements over 1s.

  - **angle_max_error**: The angle at which the maximum deviation occurred.

  - **percent_of_sigma**: Signed distance from the value of the average_std_error to the 1sigma level set of the pdf.

  - **p_value**: The p-value of the sensor noise consistency test.

  - **reject_model**: A boolean indicating if the sensor noise specs are consistent with the observed noise.


.. important::
  Currently, the expected sensor measurement covariance for 2D laserscans is 1m. This will be configurable in future versions of the RDM.


Signal Health
-------------

The signal health module is responsible for monitoring the health of the various signals between the various sub-systems like timeouts, bounds on signals and rates, NaNs, incorrect sizes etc... If critical signals expected by the RAM are not healthy, the RAM may crash which may lead to collisions.

The associated metric is published at 1hz, and is an aggregate of the signal data received over that period.

This module publishes the following metrics:

- **signal_health**: The health of the signals in the system. The important fields of the associated message are:

  - **timeout**: A boolean indicating if any of the signal has timed out.

  - **sample_size**: The number of samples received within the aggregation period.

  - **wrong_size**: A boolean indicating if the signal received has an unexpected size.

  - **bad_timestamp**: A boolean indicating if the signal received has a bad timestamp, i.e. a timestamp equal to 0.

  - **has_nan**: A boolean indicating if the signal received has NaN values.

  - **has_infinity**: A boolean indicating if the signal received has infinity values.

  - **has_zero**: A boolean indicating if the signal received has values exactly equal to `+0.f`

  - **has_subnormal**: A boolean indicating if the signal received has subnormal values.

  - **error_code**: An enum indicating the type of error that occurred. The possible values are: **[ok, bad_values, out_of_bounds, timeout]**

  - **norm_type**: Not available yet.

  - **norm**: Not available yet.

  - **values**: Not available yet.

  - **rates**: Not available yet.


System Health
-------------

The RAM needs to make process data in a timely manner to function properly. It is therefore important that the system running the RAM process be healthy. The System Health module is responsible for monitoring the health of the local compute by checking things like CPU and memory usage. High CPU and memory usage can lead to the RAM getting delayed in its publishing of commands, leading to collisions.

This module publishes the following metrics:

- **system_health**: The health of the system running the RDM. The important fields of the associated message are:

  - **cpu_load**: The CPU usage of the system in percentage.

  - **ram_usage**: The RAM usage of the system in percentage.

  - **disk_usage**: The used disk space of the system in percentage of total capacity.

  - **network_read**: The network read usage of the system in bytes/s.

  - **network_write**: The network write usage of the system in bytes/s.

  - **cpu_nb**: The number of CPUs available on the system.

  - **procs_nb**: The number of processes running on the system.


Incident Diagnostic
-------------------

The diagnostic module is there to aggregate the information published by the various metric modules and provide a high-level view of the robot's health. This module is useful for detecting issues that may not be apparent when looking at the individual metrics.

This module publishes the following metrics:

- **domain_status**: The high-level status of the health of the various robot components.

  - **system_status**: The status of the system running the Supervisor. The possible values are: **[ok, minor, severe, critical]**

  - **behavior_status**: The status of the robot's behavior, i.e. wether or not the robot is violating its safety constraints. The possible values are: **[ok, minor, severe, critical]**

  - **hardware_status**: The status of the robot's hardware, driven currently by the Dynamic Consistency metric. The possible values are: **[ok, minor, severe, critical]**

  - **perception_status**: The status of the robot's perception stack. The possible values are: **[ok, minor, severe, critical]**

  - **control_status**: The status of the robot's control stack. The possible values are: **[ok, minor, severe, critical]**


- **incidents_log**: A stream of incident logs. The important fields of the associated message are:

  - **name**: The name of the incident.

  - **detail**: The details of the incident.

  - **in_progress**: A boolean indicating if the incident is still in progress.

  - **start_time**: The time the incident started.

  - **domain**: The domain of the incident. The possible values are: **[behavior, system, hardware, perception, control]**


Output modules
==============

The RDM uses **output modules** to make the metrics available to the user. Currently, the RDM ships with a single output module to publish all metrics to :ref:`reference_ros_topics`.

.. note::
  The RDM is here again designed to be modular and extensible. For more information on what other types of output modules 3Laws can offer, please `Contact 3Laws`_.
