Quick Start
===============

Before starting the supervisor be sure to have your ROS environment correctly set up and sourced.

.. code-block:: bash

  source /opt/ros/<DISTRO>/setup.sh

To launch the Supervisor you can use the following command:

.. code-block:: bash

  ros2 launch lll_rdm rdm.launch.py

You can add the following command line options:

- **use_sim_time**: <Boolean, default: false> Specify if the RDM use the ros time or the machine time
- **config_filename**: <String> Path to the config file if not in the */opt/3lawsRoboticsInc/robot_diagnostic_module/config* folder.
- **robot_id**: <String> name of your robot
- **log_level**: <trace|debug|info|error, default: info> Log level of the RDM
- **dry_run**: <Boolean> Start the RDM, check the config file and turn off without error if everything has been launched correctly

By default the supervisor will use the ros time and the config file in /opt/3lawsRoboticsInc/robot_diagnostic_module/config/rdm_config.yaml.
You can change this default folder by setting the environment variable `LLL_CONFIG_FOLDER` to the desired path.
