Launch the Control Panel
===============

The control panel is installed as a systemd service.
To start the control panel, use the following command:

.. code-block:: bash

  sudo systemctl start lll_control_panel

and to stop it:

.. code-block:: bash

  sudo systemctl strop lll_control_panel

Also you can start it manually with the following command:

.. code-block:: bash

  python3 /opt/3lawsRoboticsInc/control_panel/server.py

You can add the following command line options:

- **-p or --port**: <number, default: 8000> Specify the port to use

You can specify the emplacement of the configuration file by setting the environment variable `LLL_CONFIG_FOLDER`
