Control Panel
========================

.. contents:: Table of Contents
   :depth: 2
   :local:

The **Control Panel of 3LawsRobotics** is a helper tool to help the configuration of the 3Laws Robotics supervisor.
It provides a web interface to configure the 3Laws Robotics supervisor and visualize the robot's state.
Currently in alpha version, it runs on your local network and can be accessed from any device with a web browser.
The backend is written in Python and uses the CGI protocol (Common Gateway Interface) to communicate with the web interface.

In order to get your account setup and use freely this RDM, please contact support@3lawsrobotics.com

This product is still at a alpha development stage. New features are developed every month and some breaking changes can still happened.
The 3LawsRobotics Team is available to answer your question.

**For now only a ROS1, ROS2 interfaces are configurable via this control panel.**

Launch the Control Panel
------------------------

The control panel is installed as a systemd service.
To start the control panel, use the following command:

.. code-block:: bash

  sudo systemctl start lll_control_panel

and to stop it:

.. code-block:: bash

  sudo systemctl stop lll_control_panel

Also you can start it manually with the following command:

.. code-block:: bash

  python3 /opt/3lawsRoboticsInc/control_panel/server.py

You can add the following command line options:

- **-p or --port**: <number, default: 8000> Specify the port to use

You can specify the emplacement of the configuration file by setting the environment variable `LLL_CONFIG_FOLDER`

Use the Control Panel
------------------------

The control panel is meant to be user friendly and safe explanatory.
However some detail might be useful to know.

Here is a picture of the control panel:

.. image:: data/Base_Info_Configuration.png
   :width: 800px
   :alt: Control Panel without any configuration

The control panel is divided in 3 parts:

- The Navigation bar, where you can find the main pages, a Ros Bridge WebSocket status indicator and a WebSocket parameter button.
- The main page, where you can find the configuration form and the control buttons.
- The footer, where you can find the version number and the 3LawsRobotics website link and a link to the update pages.


In order to use this control panel at its best, you need to configure the Ros Bridge WebSocket parameters.
To do so, click on the WebSocket parameter button in the navigation bar and fill the form with the correct parameters.

By default, the control panel is configured to connect to a Ros Bridge WebSocket running on the same machine, on the port 9090.
More information on the Ros Bridge WebSocket can be found following this link: `Ros Bridge WebSocket <http://wiki.ros.org/rosbridge_suite>`_.

.. image:: data/Valid_Ros_WS_params.png
   :width: 800px
   :alt: Control Panel with connected Ros Bridge WebSocket

A successful connection to the Ros Bridge WebSocket will be indicated by a green status indicator in the navigation bar.
