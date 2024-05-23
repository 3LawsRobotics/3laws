# Changelog

This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## 1.1.0 (20th May 2024)

New supported ROS topic types and various bug fix

### Fixed

- Node launching in incorrect namespace for ROS1
- Config file broken for ROS1
- Opening control panel from operation page not loading entire config
- Plots in operation tab of control panel not working with ROS1

### Added

- Support for new subscription topic types:
  - Ackerman Steering [Stamped]  [see doc here](https://wiki.ros.org/ackermann_msgs)
  - Accel [Stamped/withCovariance/withCovarianceStamped]
  - Pose [Stamped/withCovariance/withCovarianceStamped]
  - Twist [Stamped/withCovariance/withCovarianceStamped]
  - Transform [Stamped]

- Customizable filtered input (Runtime Assurance Module output) topic name, type and QoS

### Changed

- Shape selection improved with shape visualization.
- Various bug fixes including loading errors while loading the configuration.
- Better RAM printing in failure mode
- Improved contextual documentation in control panel to start rosbridge

### Removed

N/A


## 1.0.3 (7th May 2024)

- Fix omnidirectional dynamics.

### Fixed

- Omnidirectional dynamics not working when using input mask.

### Added

N/A

### Changed

N/A

### Removed

N/A


## 1.0.2 (4th May 2024)

Improve and fix objects map perception modality.

### Fixed

- Objects map perception modality that was not working in certain circumstances

### Added

- Objects now show up in RAM markers

### Changed

N/A

### Removed

N/A


## 1.0.1 (1st May 2024)

Fix documentation coherence.

### Fixed

- Renaming Supervisor parameters sections to fit with the documentation. "Copilot" renamed in Run-time Assurance Module
- Default port set to 8000 for every Control Panel entry point (server and CLI)

### Added

N/A

### Changed

N/A

### Removed

N/A


## 1.0.0 (1st May 2024)

This is the first version of the supervisor product.

### Fixed

N/A

### Added

- A ROS node that perform out of the box collision avoidance for any simple robot that has a lidar or an obstacle map.
- A Control Panel as a web server running on the robot to configure the supervisor and provide realtime feedback.
- A Command Line Client to manage the Control Panel server and check for updates.
- A documentation available [here](https://docs.3laws.io/).

### Changed

N/A

### Removed

N/A
