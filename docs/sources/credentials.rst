Credentials
===========

The *Credentials* page is shown in this image:

.. image:: data/cpanel1.png
   :width: 800px
   :alt: Configuration > Credentials: Control Panel page presenting Credentials, Robot name, and Company ID

The 3Laws Supervisor is designed to work in conjunction with a cloud-based server.  When you purchase the Supervisor, 3Laws will provide you with the
credentials to connect to the web-based server and with the official company
name that is used for your serivces.

- **Credentials**: 3Laws will provide you with an Influx Key and a ClickHouse key so that your robot can connect to the cloud-based server and can send its summarized information for display through Grafana.
- **Robot Name**: This identifier will be different for each robot where you install Supervisor.  It should contain a name you can use to identify an individual robot.
- **Company ID**: As part of the registration process, your company's name will be used to create an account on the cloud server. Please make sure that the name matches what 3Laws used to create your account.  This name is used for logging into the cloud server.

The upper-right of the display contains the Rosbridge connection status (upper right), configuration for the Rosbridge (gear next to the status), and a link to this documentation.  There is an "Update Instructions" link in the lower right that brings up a reminder of the instruction to run the installer script.

The **Save** button on each page of the Control Panel should be pressed before moving on to another page.
