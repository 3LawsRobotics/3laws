Troubleshooting
###############

.. contents:: Table of Contents
  :local:

The autocompletion capabilities fails to find topics:
======================================================

If the Rosbridge web server is connected to the control panel (see :doc:`User guide > Control Panel <user_guide/control_panel>` for installation and connection details), topics don't appear (only `rosout` and `client_count`) try the following:

- Check if the Rosbridge web server is connected to the control panel. You can do this by checking top right corner of the control panel. If the Rosbridge web server is connected, you should see a green icon.
- Click on the reload button in the top right corner of the topic form (green circular arrow).
- Stop the Rosbridge web server, source your ROS workspace and start the Rosbridge web server again.
- Check the Rosbridge web server log for any errors.

Messages appear in standard out reporting that QP fails to solve
================================================================

If the Supervisor filter rate is set too fast, the processor might not have enough time to finish the QP problem.  The delay may also occur if the laser scan has too many points.  The best approach is to reduce the rate at which the filter solves to see if that resolves the problem.

Platform fails to move even when there are no objects nearby
============================================================

Try launching Supervisor from a terminal to see what error messages appear. Supervisor will typically color fatal errors with "red" text.  If a critical error occurs, Supervisor will stop operating.  None of the commands will get forwarded to the platform.

Supervisor may also keep the platform stationary if the LIDAR data or the list of obstacles is not delivering data quickly enough.  This design ensures that if Supervisor does not have sufficient information about surrounding collision candidates, it does not allow the platform to proceed.  A good strategy is to start up the Operations tab for Control Panel (with Rosbridge) and see if any of the input blocks (desired input, state, perception) is a color other than green.

Another cause of platform failing to move is that the topic routing is not correct.  The ROS ``rqt`` tool with the Node Graph can visually display the routing well.  Verify that the desired inputs (e.g. cmd_vel_plan) are coming into Supervisor, and that Supervisor's outputs (e.g. /lll/ram/filtered_input) is properly mapped to the signal that the lower layer is expecting (e.g. cmd_vel).

My robot doesn't move when I send a command
============================================

Make sure in that in the :ref:`Operation <control_panel_ops>` tab of the Control Panel everything is green. If not, check the logs for errors.

The Operation tabs show that everything is OK but the robot still doesn't move
=================================================================================

Make sure that your robot is receiving the command. You can use rqt_graph to check the topic connections and topic echo to check if the command is being sent.
