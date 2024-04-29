FAQ
###

.. contents:: Table of Contents
  :local:

How does the 3Laws Supervisor work?
===================================

Supervisor depends on a **kinematic/dynamic model** of the vehicle, a measure of the **distance to the nearest object(s)**, and the current **requested input** to the system.

In simple terms, Supervisor calculates the likelihood that the current command will drive the vehicle to come too close to the nearest objects faster than the vehicle can be pushed away from them.

If the measure indicates an **imminent violation** of the requested margin, Supervisor will find the motion command that is quadratically **closest to the one being requested** but yet keeps the vehicle from colliding. When no collision is predicted, Supervisor **forwards the requested input** directly as output.

.. note::
   Supervisor does not modify the requested input unless a collision is detected.

Supervisor also support several **fault handling** options. Supervisor will monitor the regularity of receipt of the obstacle list or LaserScan input,of the state, and of the requested input. If any of these signals drops out or fails to meet criteria like number of scan points, Supervisor can either stop forwarding the requested input or can send zeroes - to stop motion of the vehicle.

What are CBFs?
==============

Control barrier functions are a **mathematical formulation** to constrain a physical system in motion to stay within a desired region of operation. By commanding the system back into what is defined as the desirable (or “invariant set”) region, certain properties similar to **stability and safety** can be provided. **In theory** these properties are guaranteed as long as the system responds according to the **mathematical model** that is being used.

The `CBF` itself is a mathematical function that measures the distance from the system’s current condition to a boundary. The objective is to keep the system within the boundary. The definition of the boundary or barrier can be built to meet a **wide variety of goals** including collision avoidance, geo-fencing, falling over, exceeding angles-of-attack, violating accelerations, and multiple others.
One of the main advantages of this formulation is that multiple objectives can be applied simultaneously.

Operation of the CBF technology requires both the definition of the barrier function and creation of a **remediation strategy**. The remediation is an approach to command the system back into a region where the function has the desirable value. Typically, there will be a large collection of commands that can maintain the system’s state within the desirable regions, so a choice of the “best” one needs to be made. The implementation in Supervisor sets up a **quadratic programming (QP) problem** to obtain this solution. This optimization means that the system will come **as-close-as-possible** to the motion that is being requested while still avoiding the undesirable outcome like a collision.

3Laws was founded by Aaron Ames and some of his team members who are responsible for creating the concept and developing/successfully **demonstrating the implementations** in a variety of systems including walking robots, wheeled vehicles, autonomous trucks, quad-copters, watersport boats, and commercial/military aircraft.

What is needed in order to implement a CBF?
===========================================

CBFs depend on a mathematical model of the physical (dynamical) system of interest. Often, a **reduced-order model can be used** without significant loss of performance/guarantees. Reduced order models result in lower computational loads. However, higher model precision and accuracy are desirable because margins for **uncertainty** can be reduced.

Next, CBFs depend on the desired barrier function itself. For collision avoidance, the barrier function could, for example, be the squared distance to the object. If multiple objects are being tracked, the barrier function can be built as the **sum of the distances to these objects**. This is a great feature because one can then use a **point-cloud to represent the objects**, and a scan can be done through the point cloud, always responding to the worst-case (closest) point.

Then, beyond the need for computational equipment, the only remaining needs are the desired commands that drive the system, the state of the system (if there are system-state variables in the barrier function) and the external measurements (like obstacles) arriving in real-time.

How are CBFs implemented in 3Laws Supervisor? What Makes 3Laws Supervisor so performant? **Provable Math**:

- Supervisor Rate of change is based on the barrier function \\(h\\), which must satisfy:  :math:`\dot{h} \geq \alpha \cdot h`
- Solution depends on applied actions \\((u)\\) and the system model: :math:`\frac{dx}{dt} = f(x) + g(x)\cdot u`
- Solution process:

   - Compute the expression for \\(h\\) to give sets of linear constraints.
   - Solve the constraints (through a `QP`) to produce modified commands that are closest to the original but meet the constraints.
   - Stopping the vehicle or changing direction are typical optimal solutions.

- Expanded Capabilities:

   - Multiple objects/barriers.
   - Noise and Uncertainties methodically handled.
   - Aggressive/Conservative Qualities can be adjusted.

- Updates by the planner immediately produce a new, modified \\(u\\).

This approach generalizes for drones, quadrupeds, aircraft, AMRs, manipulators autonomous vehicles, or anything that moves and has fly-by-wire controllers.

Does Supervisor use any methods other than CBFs to deliver its functionality?
=============================================================================

Supervisor goes beyond just CBFs in order to provide a more practical solution. The way the Supervisor **filtering** is implemented provides a really convenient path to also provide capabilities like bringing the vehicle to a “failsafe” configuration if problems are detected with incoming data.

Supervisor monitors the three primary data streams (system state, desired inputs, obstacle/sensor) and if any of these states does not meet the **requirements** such as regular arrival time, Supervisor can switch to a fail-safe state (e.g. send stopping commands).

.. note::
  Supervisor can also monitor additional variables that have discrete states (e.g. “starting”, “running”, “stopped”), and can cause a change to failsafe state if one or more of these monitored variables has an undesirable value.

What are some examples of 3Laws Supervisor applications? In other words, how can Supervisor be used in the real world?
======================================================================================================================

Currently, Supervisor only supports collision avoidance for **omni-directional**, **differential-drive**, and **front-steering** vehicles.

Supervisor **Pro** offers collision avoidance, geofencing, system stability, and other operation constraint satisfiers. Specifically, Supervisor Pro can add the necessary control to your system to satisfy any performance constraint as long as your systems’ sensors provide the relevant real-time metrics.

What is the difference between Supervisor and Supervisor **Pro**?
=================================================================

Supervisor is a fast-to-install, low-configuration collision avoidance software component for omni-directional, differential-drive, and single-track vehicles controlled by ROS or ROS2.

Supervisor **Pro** uses the same codebase as Supervisor but is **tailored/optimized** for specific platforms and use cases.

For example, if the system uses interfaces other than ROS, Supervisor Pro can be designed using the appropriate interfaces. More importantly, Supervisor Pro will achieve higher performance because it is built for the individual platform including considerations about its operational domain and what objectives it must accomplish.

The Control Barrier Functions and models most appropriate to the individual operations are selected and applied.

On what types of systems has 3Laws implemented Supervisor?
==========================================================

Supervisor is currently implemented on Autonomous Mobile Robots (AMRs), quadrupeds, and wheeled vehicles that are differentially-driven, front-steered, or omni--directional. Supervisor Pro has been implemented on semi trucks, military drones, fighter jets, manipulators, AMRs, inverted pendulums (Segways), speed boats, and quadrupeds.

How long has Supervisor been under development?
================================================

Supervisor has a long and proven legacy of development and industrial testing both at Georgia Tech and Caltech for over 8 years with industry leaders such as Raytheon, Ford, Wondercraft, BP, et al.
