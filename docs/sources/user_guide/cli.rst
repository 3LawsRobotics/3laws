Command Line Interface (CLI)
############################

.. contents:: Table of Contents
  :local:

The Supervisor package provides a CLI that can be used to interact with the Supervisor.

The command for the CLI is `3laws`. It can be used to start, stop, and restart the control panel service used for the configuration of the Supervisor.

The CLI provides also a command to check for updates of the Supervisor. These updates still have to be installed manually. See :ref:`Installation` section.


Run the following command to see the available commands:

.. code-block:: bash

  3laws --help-all

Run Control Panel
==================

After installation the command-line interface (CLI) can be used to start the Control Panel:

.. code-block:: bash

  3laws control-panel run

.. important::
  This command will start the Control Panel in the foreground.  Appending an ampersand ``&`` can send it to the background freeing up the terminal for other operations.

Control Panel service
=====================

If a service is preferred for running the Control Panel in the background, use the following command:

.. code-block:: bash

  3laws control-panel autostart enable

This will create a user service. This one will be started automatically when the system boots up. The Control Panel will be available at `http://localhost:8080`. To change the communication port, use the following command:

.. code-block:: bash

  3laws control-panel autostart enable --port <PORT>

To turn off the service so that the Control Panel service is removed from the system:

.. code-block:: bash

  3laws control-panel autostart disable

Check for available updates
===========================

You can check for available updates with the following command (requires an active internet connection):

.. code-block:: bash

  3laws check-update


.. warning::
  The update process is not yet automated. The user has to download the new version and install it manually.
