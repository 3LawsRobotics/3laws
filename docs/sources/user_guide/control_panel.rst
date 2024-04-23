Control Panel
#####################################

.. _control_panel_config:

Configuration
*************
.. toctree::

   1. License key and Robot Name <configuration/credentials>
   2. Robot Shape/Kinematics and Command Inputs <configuration/robot_model>
   3. Copilot tuning and Additional States to Monitor <configuration/supervisor>
   4. Localization state and Constraints on State <configuration/localization>
   5. Sensor Configuration for Collision Avoidance <configuration/perception>


.. _control_panel_ops:

Operations
**********

.. image:: ../data/cpanel6.png
   :align: center
   :width: 600px
   :alt: Operations page showing a configured robot that does not yet have sensor or planning data.


In the image above, the Supervisor is operational and all the Copilot is configured to be active as indicated by the arrows between them. When data is not yet available (e.g. rosbridge connection is not operational) the boxes appear as golden. If the component has not yet initialized, the background for the box is blue, while if there is a detected error, the box is red. Proper operation is indicated by a green-colored box.

The lower section of the panel is showing strip charts. The categories that are currently displayed represent:

* the State Safeness - the barrier function value. When this value goes to zero or below zero, the system is evaluated as being in a collision state.

* the Input Modification status - When this value is zero, the copilot is not modifying the input from the autonomy stack. That is, the filtering is in passive mode. When this value is non-zero, it means that the copilot is actively modifying the commanded input.

* Latest logs - shows the most recently detected events.
