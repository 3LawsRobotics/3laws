Credentials
==============

Credentials are used to connect to the databases to store the data.
Here is the structure and the field detail.

::

  credentials:
    company_id: Company name
    robot_id: Robot Name
    influx_credentials:
      database_token: INFLUX token provided by 3lawsRobotics
    clickhouse_credentials:
      username: Company name
      password: CLICKHOUSE password provided by 3lawsRobotics
