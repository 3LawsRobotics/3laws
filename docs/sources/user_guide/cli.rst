Command Line Interface (CLI)
############################

The Supervisor package provides a CLI that can be used to interact with the Supervisor.

The command for the CLI is `3laws`. It can be used to start, stop, and restart the control panel service used for the configuration of the Supervisor.

The CLI provides also a command to check for updates of the Supervisor. These updates still have to be installed manually. See :ref:`Installation` section.


Run the following command to see the available commands:

.. code-block:: bash

  3laws --help-all

After installation the command-line interface (CLI) can be used to start the Control Panel:

.. code-block:: bash

  3laws control-panel run

If a service is preferred for running the Control Panel in the background, use the following command:

.. code-block:: bash

  3laws control-panel autostart enable

This will create a user service. This one will be started automatically when the system boots up. The Control Panel will be available at `http://localhost:8080`. To change the communication port, use the following command:

.. code-block:: bash

  3laws control-panel autostart enable --port <PORT>

To turn off the service so that the Control Panel service is removed from the system:

.. code-block:: bash

   3laws control-panel autostart disable

The Control Panel can also display a summary of operational conditions, but this capability requires a rosbridge server. To install and start a rosbridge
server (where <rosdistro> is replaced with the version of ROS on the system):

.. code-block:: bash

  sudo apt-get install ros-<rosdistro>-rosbridge-server
  ros2 run rosbridge_server rosbridge_websocket

This will provide a websocket server at **`ws://localhost:9090`** that the control panel can connect to in order to retrieve topics and services information.

The navigation bar of the control panel will show the status of the rosbridge server connection:

.. image:: ../data/navigation_bar_rosbridge.png
   :width: 800px
   :alt: Control Panel NavBar with ros bridge connected.
