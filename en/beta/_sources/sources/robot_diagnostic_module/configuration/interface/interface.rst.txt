Interface Configuration
========================

The interface structure is here to get precision on the interface.
Right know only ROS interfaces are available, the interface option are here to specify Quality of Service and topic types

.. code-block:: yaml

  interface:
    retimestamp: never/always/if_zero specify if the timestamp has to be set

    ros_topics_info:
      /example_topic!:
        type: sensor_msgs/LaserScan
      /example_topic2:
        type: lll_msgs/Float64VectorStamped
      /example_topic3:
        type: nav_msgs/Odometry

Supported topics type are all ROS default types and 3lawsRobotics custom types (ns: lll_msg, lll_rdm_msg)