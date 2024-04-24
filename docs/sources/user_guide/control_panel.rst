Control Panel
##############


Ros bridge connection
=====================

In order to help you at its best, the control panel can be used with a rosbridge server.

To install a rosbridge server (where <rosdistro> is replaced with the version of ROS on the system):

.. code-block:: bash

  sudo apt-get install ros-<rosdistro>-rosbridge-server

To run the rosbridge server, use the following command:

.. code-block:: bash

  ros2 run rosbridge_server rosbridge_websocket

This will provide a websocket server at **`ws://localhost:9090`** that the control panel can connect to in order to retrieve topics and services information.

The navigation bar of the control panel will show the status of the rosbridge server connection:

.. image:: ../data/navigation_bar_rosbridge.png
  :width: 800px
  :alt: Control Panel NavBar with ros bridge connected.

.. note::

  The control panel can be used without a rosbridge server, but the autocompletion for topics and the operation tab will not be available.

.. _control_panel_config:

Configuration
=============

The configurable fields are describe in the following pages.

Each of the following sections corresponds to a tab in the Control Panel. The tabs are:

.. toctree::
  :maxdepth: 1

  1. Credentials <configuration/credentials>
  2. Model <configuration/robot_model>
  3. Supervisor <configuration/supervisor>
  4. Localization <configuration/localization>
  5. Perception <configuration/perception>


The **Save** button on each page of the Control Panel should be pressed to save the current page before moving on to another page.

Throughout this documentation, a red asterisk (*) indicates a *required* field.

.. note::

  The configuration is saved in the `~/.3laws/config/supervisor.yaml` file.

.. warning::

  In order to get topic autocompletion from the rosbridge server, your stack needs to be running and publishing topics.

.. _control_panel_ops:

Operations
==========

.. image:: ../data/cp_operation.png
  :align: center
  :width: 600px
  :alt: Operations page showing a configured robot that does not yet have sensor or planning data.

|

In the image above, the Supervisor is operational and all the Run-time Assurance Module is configured to be active as indicated by the arrows between them. When data is not yet available (e.g. rosbridge connection is not operational) the boxes appear as golden. If the component has not yet initialized, the background for the box is blue, while if there is a detected error, the box is red. Proper operation is indicated by a green-colored box.

The lower section of the panel is showing strip charts. The categories that are currently displayed represent:

* the State Safeness - the barrier function value. When this value goes to zero or below zero, the system is evaluated as being in a collision state.

* the Input Modification status - When this value is zero, the Run-time Assurance Module is not modifying the input from the autonomy stack. That is, the filtering is in passive mode. When this value is non-zero, it means that the Run-time Assurance Module is actively modifying the commanded input.

* Latest logs - shows the most recently detected events.

.. warning::

  In order to make this page work, the rosbridge server needs to be connected to the control panel.
