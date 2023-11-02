Robot Description
==================

This part is the most important one. The more accurate the information provided here, the more insightful the RDM will be.

Each sub structure is optional but once a substructure is declared, all its fields must be filled (unless Optional is specified)



.. contents:: Table of Contents
   :depth: 2
   :local:


Robot model
-----------

.. code-block:: yaml

    robot_type: <String> See available robot type section for more details

    kinematic_model: # Kinematic model of the robot
      base_frame_id: <String> # Name of the base frame of the robot
      geometry: # Geometry of the robot
        shape_type: <Enum point|box|sphere|ellipsoid|capsule|cone|cylinder|mesh> # Type of the geometry shape (should be one of [point, box, sphere, ellipsoid, capsule, cone, cylinder, mesh])
        shape_params: # Parameters for the chosen shape_type
          x_size: <Float> #in Meter
          y_size: <Float> #in Meter
          z_size: <Float> #in Meter
        pose: # [optional] Pose of shape w.r.t base frame
          translation: <Float array[3]> # [optional] Translation w.r.t base frame [x,y,z], default [0,0,0]
          rotation: <Float array[4]> # [optional] Rotation quaternion w.r.t base frame [w,x,y,z], default [1,0,0,0]

    dynamical_model: # [optional] Behavior model of the robot
      model_type: <String> # Type of dynamical model for selected robot_type
      state_domain: # Region of the state space in which the dynamical model has been validated
        simple: <Boolean> # [optional], If true, assumes M is an identity matrix (default: false)
        n_cstr: 3 # [optional if simple==true] Number of rows in M
        M: # [optional if simple==true] Either a single sequence representing the diagonal of M, or a sequence of the rows of M
          - [1., 0., 0., 0., 0.]
          - [0., 1., 0., 0., 0.]
          - [0., 0., 1., 0., 0.]
        # Defined as a a convex polytope S={x in Rn | lb <= M.x <= ub}
        ub:
          <Float array[n_cstr]> # If simple==true, must have same size as state of model_type,
          # otherwise must have size n_cstr
        lb:
          <Float array[n_cstr]> # If simple==true, must have same size as state of model_type,
          # otherwise must have size n_cstr

        input_domain: # Region of the input space accessible for control purposes and in which the dynamical model has been validated
          # Same representation as 'state_domain'
          ub:
            <Float array[model_input_size]> # If simple==true, must have same size as input of model_type,
            # otherwise must have size n_cstr
          lb:
            <Float array[model_input_size]> # If simple==true, must have same size as input of model_type,
            # otherwise must have size n_cstr
          simple: <Boolean>

      process_noise_covariance:
        <Float array[model_state_size]> # Either a single sequence representing the diagonal of the process noise covariance matrix,
        # or a sequence of its rows. Must have same size as state of model_type
      model_param: # [variant] Parameters for the chosen model_type, as example here for the bicycle
        wheel_dx: 1.
        origin_dx: 1.
        tau_vel: 1.
        tau_steer: 1.


Available shape types:
^^^^^^^^^^^^^^^^^^^^^^

The kinematic model of the robot defines its geometry. The geometry is described by a base frame,
a shape, and the pose of that shape in the base frame. We currently support the following shapes:

- Point:
  shape_params: ~

- Box:

  shape_params:
    x_size: <Float>  Total length of the box along x axis
    y_size: <Float>  Total length of the box along y axis
    z_size: <Float>  Total length of the box along z axis_mask

- Sphere:

  shape_params:
    radius: <Float>  Radius of the sphere

- Ellipsoid

  shape_params:
    radius_x: <Float>  Semi x-axis length
    radius_y: <Float>  Semi y-axis length
    radius_z: <Float>  Semi z-axis length

- Capsule:

  shape_params:
    radius: <Float>  Radius of the capsule
    length: <Float>  Length of the capsule

- Cone:

  shape_params:
    radius: <Float>  Radius of the cone
    length: <Float>  Length of the cone

- Cylinder:

  shape_params:
    radius: <Float>  Radius of the cylinder
    length: <Float>  Length of the cylinder

- Mesh:

  shape_params:
    mesh_file: /opt/mesh.stl  Path to mesh file
    mesh_type: stl  Type of mesh file, available options: [stl]
    mesh_units: mm  [optional] Units of the mesh file, available options: [mm, cm, dm, m, dam, hm, km, mi, nm, yd, ft, in], default: m


Available robot type:
^^^^^^^^^^^^^^^^^^^^^

Each dynamical model type has its own set of states, inputs, and parameters:

- mobile_robot:

  - differential_drive: Dynamical model for a rigid body over SE2 with first order tracking response of longitudinal and rotational body velocities

    states: [x,y,yaw,vx_body_actual,wz_body_actual]

    inputs: [vx_body_command,wz_body_command]

    parameters:

    - tau_vel -> time constant of the 1st order tracking response in linear velocity (1/s) (must be strictly positive)
    - tau_yaw_vel -> time constant of the 1st order tracking response in angular velocity (1/s) (must be strictly positive)

  - bicycle: Dynamical model for a 2 wheels or 4 wheel but with coupled front wheel steering vehicle over SE2, with first order tracking response of steering angle and origin velocity magnitude.

    states: [x,y,yaw,||v_body||_actual,steering_angle_actual]

    inputs: [||v_body||_command,steering_angle_command]

    parameters:

    - wheel_dx -> Distance between front and back wheels (m) (must be strictly positive)
    - origin_dx -> Position of vehicle's origin w.r.t back wheels (m) (must be positive)
    - tau_vel -> time constant of the 1st order tracking response in linear velocity (1/s) (must be strictly positive)
    - tau_steer -> time constant of the 1st order tracking response in angular velocity (1/s) (must be strictly positive)



Robot Autonomy stack
--------------------

.. code-block:: yaml

    mission_manager:
      extra_topics: ~ # [optional]
      process_name: ~ # [optional]
      finite_states: # Finite states of the robot
        - interface_id:
            /status # Name of the ros topic.
            # Supported types: [(default) std_msgs/String, std_msgs/FloatXX, std_msgs/Bool, std_msgs/Char, std_msgs/Byte, std_msgs/IntXX, std_msgs/UIntXX]
          sender_id: state_machine # Display name for sender of this state
          state_id: status # Identifier for this state, "<sender_id>.<state_id>" must form a UNIQUE identifier among all signals
          signal_min_rate: 1s
        - interface_id: /search_mode
          sender_id: state_machine
          state_id: search_mode
          signal_min_rate: 1s

    path_planning:
      extra_topics: ~ # [optional]
      process_name: ~ # [optional]
      paths:
        - interface_id:
            /desired_path # Name of the ros topic.
            # Supported types: [(default) lll_msgs/Trajectory, nav_msgs/Path, trajectory_msgs/JointTrajectory]
          path_id: main_path # Display name for this path, must be UNIQUE among all paths
          trajectory_state_size: 7 # Size of the trajectory state vector
          signal_min_rate: 1min # Maximum time without receiving data before signal is considered timed out
          # state_mask:
          #   [0, 1, 2, 3, 4, 5, 6] # [optional] If the path only corresponds to a subset of the state_estimation vector,
          #   # use this mask to extract the relevant data : trajectory_state[i] = state_estimation[state_mask[i]].
          #   # Must be of size 'trajectory_state_size', and not contain indices greater than state_estimation.state_size.
          #   # If not specified or null, will be [0, ..., trajectory_state_size-1]
          tracking_error_bounds: ~ # [optional] Bounds on controller's tracking error : path_state - actual_state


Robot perception
----------------

.. code-block:: yaml

  sensors:
      extra_topics: ~ # [optional]
      process_name: ~ # [optional]
      batteries: [] # Coming soon!
      cameras: [] # Coming soon!
      gps: [] # Coming soon!
      imus: [] # Coming soon!
      laserscans: # Planar laser scanners
        - interface_id:
            /laserscan_1_topic # Name of the ros topic.
            # Supported types: [(default) sensor_msgs/LaserScan]
          sensor_id: laserscan_1 # Display name for this laserscan, must be UNIQUE among all laserscans
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          specs:
            n_rays: 1000 # Expected number of rays in the laserscan
            angle_min: -3.14 # Minimum ray angle
            angle_max: 3.14 # Maximum ray angle
            range_min: 0. # Minimum ray range
            range_max: 1000. # Maximum ray range
            noise_one_sigma: 0.025 # Expected standard_error of the sensor (given by the manufacturer, often like: precision = +-2sigma)
          transform: # Specification of frame w.r.t which the measurement is expressed
            parent_frame_id: robot # Id of parent frame
            pose: # [optional] Pose w.r.t parent frame
              translation: [0., 0., 0.] # [optional] Translation w.r.t parent frame [x,y,z], default [0,0,0]
              rotation: [1., 0., 0., 0.] # [optional] Rotation quaternion w.r.t parent frame [w,x,y,z], default [1,0,0,0]
      lidars: [] # Coming soon!
      loadcells: # Force and torque measurement sensor, 6 axis by default
        - interface_id:
            /end_effector_wrench # Name of the ros topic.
            # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
          sensor_id: end_effector_loadcell # Display name for this loadcell, must be UNIQUE among all loadcells
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          transform: # Specification of frame w.r.t which the measurement is expressed
            parent_frame_id: robot # Id of parent frame
            pose: # [optional] Pose w.r.t parent frame
              translation: [0., 0., 0.] # [optional] Translation w.r.t parent frame [x,y,z], default [0,0,0]
              rotation: [1., 0., 0., 0.] # [optional] Rotation quaternion w.r.t parent frame [w,x,y,z], default [1,0,0,0]
          # axis_mask: # [optional] Define which of the 6 force/torque axes in SE3 the loadcell signals correspond to: [Fx, Fy, Fx, Mx, My, Mz].
          #   # If not specified or null, assumes all 6 axes.
          #   # Cannot be empty or longer than 6. Index must be between 0 and 5 included.
          #   [0, 5] # Corresponds to a 2 axis loadcell [Fx,Mz]
          noise_one_sigma: [1., 1., 1., 1., 1., 1.] # Noise characteristics of loadcell axes. Must have same size as axis_mask
          bounds: ~ # [optional]

    perception:
      obstacles: # [optional] List of obstacles
        interface_id: /obstacles # Name of the ros topic. # Supported types: [(default) lll_msgs/ObjectArray]
        signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
        meshes: # List of meshes to be loaded
          []
          # - id: sphere # Mesh identifier, must be UNIQUE among all meshes
          #   data: # Mesh data
          #     mesh_file: sphere.stl # Path to mesh file
          #     mesh_type: stl # Type of mesh file
          #     mesh_units: mm # Unit of mesh file

    localization:
      extra_topics: ~ # [optional]
      process_name: ~ # [optional]
      state_estimation: # [optional]
        interface_id:
          /state # Name of the ros topic.
          # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
        signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
        state_size: 5 # Size of the state vector
        # mask:
        #   [0, 1, 2, 3, 5] # [optional] If only a subset of the vectorized message actually constitute the state vector
        #   # use this mask to extract the relevant data : state[i] = msg_vectorized[mask[i]].
        #   # Must be of size 'state_size', and not contain indices greater than the size of vectorized message.
        #   # If not specified or null, will be [0, ..., state_size-1]
        bounds: ~ # [optional]

      odometry:
        - interface_id:
            /odom_node_0_topic # Name of the ros topic.
            # Supported types: [(default) nav_msgs/Odometry]
          odom_id: odom_node_0 # Display name for this odometry source, must be UNIQUE among all odometry
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          se2_only: true # [optional] Consider only SE2 projection of pose and twist (default: false)
          position_bounds: # [optional] Position part of the odometry. Components are [x,y,z] or [x,y] if se2_only==true
            norm_type: none
            norm_upper_bound: 1.
            norm_lower_bound: 0.
            upper_bounds: [1., 1.]
            lower_bounds: [-1., -1.]
            rates_upper_bounds: []
            rates_lower_bounds: []
          orientation_bounds: ~ # [optional] Same fields as position. Components are [roll,pitch,yaw] or [yaw] if se2_only==true
          velocity_linear_bounds: ~ # [optional] Same fields as position. Components are [vx,vy,vz] or [vz,vy] if se2_only==true
          velocity_angular_bounds: ~ # [optional] Same fields as position. Components are [wx,wy,wz] or [wz] if se2_only==true


Robot control
--------------

.. code-block:: yaml

  control:
    extra_topics: ~ # [optional]
    process_name: ~ # [optional]
    setpoint_tacking_controllers: # PID like controllers
      - controller_id: velocity_controller # Display name for this controller, must be UNIQUE among all controllers
        state_size: 1 # Size of controller setpoint
        input_size: 1 # Size of control input computed by controller
        desired_state:
          interface_id: /controller_cmd_topic # Name of the desired state ros topic.
          # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
          mask: [0] # [optional] If only a subset of desired_state_topic_id vector is actually used by controller,
          # use this mask to extract the relevant data : desired_state_used[i] = desired_state_received[desired_state_mask[i]]
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          bounds: ~ # [optional] Bounds on desired state

        actual_state:
          interface_id: /controller_state_topic # Name of the actual state ros topic.
          # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
          mask: ~ # [optional] Same as desired_state_mask
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          bounds: ~ # [optional] Bounds on actual state

        control_input:
          interface_id: /controller_input_topic # Name of the control input ros topic.
          # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
          mask: ~ # [optional] Same as desired_state_mask
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          bounds: # [optional] Bounds on controller's control input
            norm_type: none
            norm_upper_bound: 1.
            norm_lower_bound: 0.
            upper_bounds: []
            lower_bounds: []
            rates_upper_bounds: [1.]
            rates_lower_bounds: [-1.]
        tracking_error_bounds: # [optional] Bounds on controller's tracking error : desired_state - actual_state
          norm_type: none
          norm_upper_bound: 1.
          norm_lower_bound: 0.
          upper_bounds: [1.]
          lower_bounds: [-1.]
          rates_upper_bounds: []
          rates_lower_bounds: []

    actuators: # Robot actuation
      combined: # Combined actuation vector
        interface_id:
          /control_input # Name of the ros topic publishing the complete robot actuation vector.
          # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
        input_size: 3 # Size of the combined input vector
        signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
        bounds: ~ # [optional]
        # mask:
        #   [0,1,3] # [optional] If only a subset of the vectorized message actually constitute the combined input vector
        #   # use this mask to extract the relevant data : input[i] = msg_vectorized[mask[i]].
        #   # Must be of size 'input_size', and not contain indices greater than the size of the vectorized message.
        #   # If not specified or null, will be [0, ..., input_size-1]

    supervisors: # 3Laws AI Supervisors
      - interface_id: /main_supervisor_topic # Name of the supervisor data ros topic.
        supervisor_id: main_supervisor # Display name for this supervisor, must be UNIQUE among all supervisor
        signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out


Extras
--------

.. code-block:: yaml

    extras:
      passthrough_metrics: # Generic passthrough for scalar metric signals
        - interface_id:
            /metric_1_topic # Name of the ros topic.
            # Supported types: [(default) std_msgs/Float64, std_msgs/Float32, std_msgs/Bool, std_msgs/Char, std_msgs/Byte, std_msgs/IntXX, std_msgs/UIntXX]
          metric_id: metric_1 # Display name for this metric, must be UNIQUE among all passthrough metrics
          metric_group_id:
            position # [optional] Group this signal belongs to.
            # Metrics of the same group are plotted on the same graph in 3laws.app

      clocks:
        - interface_id:
            /custom_clock # Name of the ros topic.
            # Supported types: [(default) rosgraph_msgs/Clock]
          clock_id: my_clock # Display name for this clock, must be UNIQUE among all clocks
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out

      signals: # Generic floating point multidimensional signal values sanity and bounds checking
        - interface_id:
            /test_signal_topic # Name of the ros topic.
            # Supported types: [(default) lll_msgs/Float64VectorStamped, any other vectorizable type (see bottom of this file)]
          sender_id: test_signal_node # Display name of sender node
          signal_id: test_signal # Display name of this signal, "<sender_id>.<signal_id>" must form a UNIQUE identifier among all signals
          signal_size: 1 # Size of this signal
          signal_min_rate: 1s # Maximum time without receiving data before signal is considered timed out
          # mask:
          #   [2] # [optional] If only a subset of vectorized message actually constitute the signal vector
          #   # use this mask to extract the relevant data : signal[i] = msg_vectorized[mask[i]].
          #   # Must be of size 'signal_size', and not contain indices greater than the size of the vectorized message.
          #   # If not specified or null, will be [0, ..., signal_size-1]
          bounds: ~ # [optional]

      nodes: # Generic node health checking metric
        - node_id: test_node # Display name of node, must be UNIQUE among all nodes
          # text_log_interface_id:
          #   /test_node_log # [optional] Name of the ros topic publishing log info for that node.
          #   # Supported types: [(default) rcl_interfaces/Log]
          # process_name: # [optional]
          #   test_node_exec.
          topics: # List of topics published by the node (only available in ros2 humble and up)
            - interface_id:
                /test_node_topic_1 # Name of the ros topic. Associated 'interface.message_type_map.<interface_id>' must be specified.
                # Supported types: [builtin_interfaces/*, geometry_msgs/*, lll_msgs/*, nav_msgs/*, rcl_interfaces/*, rosgraph_msgs/*, sensor_msgs/*, std_msgs/*, trajectory_msgs/*, visualization_msgs/*]
              topic_id: test_node_topic_1 # Display name for this topic, must be UNIQUE among all topics of each node
              signal_min_rate: 1s # Maximum allowed duration without receiving data
        - node_id: rosout
          text_log_interface_id: /rosout # If equal to '/rosout', uses 'name' field of incoming rcl_interfaces/Log message as node_id for text_log message
          topics: []