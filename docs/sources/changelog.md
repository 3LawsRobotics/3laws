# Change Log

This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## 1.0.3 (7th May 2024)

Fix omnidirectional dynamics not working when using input mask.

### Added

N/A

### Changed

N/A

### Removed

N/A

## 1.0.2 (4th May 2024)

Fix objects map perception modality that was not working in certain circumstances.

### Added

- Objects now show up in RAM markers

### Changed

N/A

### Removed

N/A

## 1.0.1 (1st May 2024)

Fix documentation coherence.

### Added

N/A

### Changed

- Renaming Supervisor parameters sections to fit with the documentation. "Copilot" renamed in Run-time Assurance Module
- Default port set to 8000 for every Control Panel entry point (server and CLI)

### Removed

N/A


## 1.0.0 (1st May 2024)

This is the first version of the supervisor product.

### Added

- A ROS node that perform out of the box collision avoidance for any simple robot that has a lidar or an obstacle map.
- A Control Panel as a web server running on the robot to configure the supervisor and provide realtime feedback.
- A Command Line Client to manage the Control Panel server and check for updates.
- A documentation available [here](https://docs.3laws.io/).

### Changed

N/A

### Removed

N/A
