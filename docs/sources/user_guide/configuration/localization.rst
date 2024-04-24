Localization
############

Dialogs to connect to the state information provided for the robot and for configuring alerts based on state are on the Configuration > Localization page. contains configuration entries for both the monitoring and input filtering components.

- **Localization topic**: The connection to the ROS state topic is configured in this area. As with input commands, the message topic name, message topic type, expected message topic quality, and expected message rates are specified. If the message quality fails or the message receipt rate is not met, the monitor will issue alerts, and the Copilot will switch to the Failsafe strategy. The mask needs to be customized if the localization topic does not respect the standard ROS message. The index mapping from the input vector to the expected states (x-position, y-position, yaw) needs to be set.  The primary use of the localization message is to provide a MarkerArray for visualization mapped to a global/odom frame rather than providing just the base-frame view where the frame stays still and the outside world moves relative to it.
