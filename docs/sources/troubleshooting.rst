Troubleshooting
###############


The autocompletion capabilities do not show my topics
======================================================

If the rosbridge web server is connected to the control panel (see :doc:`User guide > Control Panel <user_guide/control_panel>` for installation and connection details), but you still don't see any of your topics (only `rosout` and `client_count`) try the following:

- Check if the rosbridge web server is connected to the control panel. You can do this by checking top right corner of the control panel. If the rosbridge web server is connected, you should see a green icon.
- Click on the reload button in the top right corner of the topic form (green circular arrow).
- Stop the rosbridge web server, source your ROS workspace and start the rosbridge web server again.
- Check the rosbridge web server log for any errors.
