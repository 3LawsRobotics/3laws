Change Log
##########

.. contents::
  :local:

This project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

1.0.1 (1st May 2024)
=====================

First patch of this version, fixing documentation coherence

Added
-------

N/A

Changed
-------

- Renaming Supervisor parameters sections to fit with the documentation. "Copilot" renamed in Run-time Assurance Module
- Default port set to 8000 for every Control Panel entry point (server and CLI)

Removed
-------

N/A


1.0.0 (1st May 2024)
=====================

This is the first version of the 3Laws Supervisor product.

Added
------

- An out-of-the-box ROS node that performs collision avoidance for simple ground-based robots that have a lidar or an obstacle map.
- A Control Panel for Supervisor configuration implemented as a web service running locally on the robot. The Control Panel also provides real-time display of operational status.
- A Command Line Client (CLI) to manage the Control Panel server and check for Supervisor software updates.
- Documentation available `here <https://3lawsrobotics.github.io/3laws/en/latest/>`_.

Changed
--------

N/A

Removed
--------

N/A
