.. _reference_ros_parameters:

ROS Parameters
##############

- ``config_filepath``: The path to the configuration file that the Supervisor should use. The default value is ``~/.3laws/config/supervisor.yaml``.

- ``robot_id``: The ID of the robot that the Supervisor is controlling. This is used to identify the robot in the logs and other messages. The default value is an empty string, in which case the ``robot_id`` field in the config file is used.

- ``log_level``: The log level that the Supervisor should use. This can be one of ``trace``, ``debug``, ``info``, ``warn``, ``err``, ``critical`` or ``off``. The default value is ``info``.

- ``dry_run``: If set to ``true``, the Supervisor will run in dry-run mode, which means that it will start and stop without sending commands to the robot, Use for validating that the configuration is valid. The default value is ``false``.

- ``log_filepath``: The path to the log file that the Supervisor should write to. If empty, the Supervisor won't write a log to disk. The default value is ``~/.3laws/log/supervisor.log``.
