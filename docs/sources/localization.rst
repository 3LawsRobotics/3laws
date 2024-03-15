Localization
============

Dialogs to connect to the state information provided for the robot and for configuring alerts based on state are on the Configuration > Localization page. contains configuration entries for both the RDM (monitor) and RTA (Copilot) components.

.. image:: data/cpanel4.png
   :width: 800px
   :alt: Configuration > Localization page where monitoring of the vehicles location/state is configured. 


- **Localization topic**: The connection to the ROS state topic is configured in this area. As with input commands, the message topic name, message topic type, expected message topic quality, and expected message rates are specified. If the message quality fails or the message receipt rate is not met, the monitor will issue alerts, and the Copilot will switch to the Failsafe strategy.  The mask needs to be customized if the localization topic is a vector of values that is not a standard ROS message.  The index in the input vector relating to the individual states (x, y, yaw) nees to be set correctly. 


- **Robot state constraints**: Limits on the absolute location (relative to the origin of the world frame) and limits on the measured rates of change (with respect to time) of the vehicle state are set in this area in order to trigger events and alerts for the monitoring function.  The "no bounds" option allows infinite travel in the respective directions or speeds.
