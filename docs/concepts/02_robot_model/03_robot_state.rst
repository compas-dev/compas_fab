.. _robot_state:

*******************************************************************************
Robot State
*******************************************************************************

While the robot model defines the intrinsic properties of the robot, the robot state
defines the modifiable (extrinsic) properties of the robot. In **COMPAS FAB**, the
robot's state consist of two components, the robot's joint configuration (represented
by a :class:`compas_robots.Configuration`) and the Robot Coordinate Frame (aka. the 
robot's base frame, represented by a :class:`compas.geometry.Frame`).

Both of these properties are set in the :class:`compas_fab.robots.RobotCellState`,
which will be :ref:`introduced later <robot_cell_state>`.

.. _configuration:

Robot Configuration
===================

The robot's configuration is a set of joint values that define the positions
of its kinematic joints. The :class:`compas_robots.Configuration` class is implemented
with a key-value structure, where the keys are the joint names and the values
are the joint values. Note that the joint values are in radians for revolute joints
and in meters for prismatic joints. The joint values can be set directly by the user
or by a motion planner, such as those offered by **COMPAS FAB**.

Full Configuration
------------------
Typically robots have more than one joint, and the configuration of the robot
is defined by the values of all the joints. This is referred to as a 'full configuration'.
Technically speaking, a full configuration is one where all the configurable joints
(aka. not passive, not fixed and not a mimic joint) are defined. 

Group Configuration
------------------
In some cases, especially for robots with higher degrees of freedom, it is useful to 
define the configuration for a subset of the joints that belongs to a planning group.
This is referred to as a 'group configuration'. In the case of motion planning,
only the joints that belong to the planning group are movable, therefore we a group
configuration is sufficient to define the robot's changing state during the motion.

The :class:`compas_robots.Configuration` class does not distinguish between
full and group configurations, but it is important for the user to understand
the difference between the two. For example, all the motion planning and visualization
functions requires a starting state as input, where the :attr:`compas_fab.robots.RobotCellState.robot_configuration`
attribute must be a full configuration. On the other hand, many of the planning 
functions return a group configuration for a more compact representation.

In order to convert a group configuration to a full configuration, the values of the
remaining joints must be defined, typically by setting them to the starting values
of the robot's state. The function :meth:`compas_robots.Configuration.merged` can be used
to merge a group configuration with a full configuration.

.. _robot_coordinate_frame_in_robot_state:

Robot Coordinate Frame
======================

The robot coordinate frame (RCF) refers to the location (pose) of the robot in the world
coordinate frame (WCF). It is default to be an identity transformation, meaning that
the robot's base frame is at the origin of the world coordinate frame.
Typically, it is not necessary to change the robot's base frame.
However, it can be useful in some cases to change the robot's base frame.

One example is when the robot controller's world coordinate system (or work coordinate
system) is defined differently from the base frame of the robot model. This happens
famously with the UR robots, where the robot manufacturer (who makes the controller)
and the ROS-Industrial community (who makes the URDF package) adopts different conventions
for the base frame of the robot. This resulted in a mismatch of target poses between the
robot controller display and the robot model in the simulation. By changing the base frame
of the robot model, the target poses can be aligned.

Another example is when the robot is mounted on a mobile platform, and such kinematics is
not modelled in the Robot Model. Instead of modifying the RobotModel, which could be
challenging for the ROS MoveIt backend, the robot's base frame can be changed by the user
to match the mobile platform's frame. This way, the robot's position is correctly defined
in the world coordinate frame.

The ability to change the RCF can also be useful for robotic setups that include external
measuring devices, such as a camera or a total station. In this case, it can be more
convenient to define the measuring device's frame as the world coordinate frame, and
position the robot in relation to the measuring device. As such, the targets observed
by the measuring device can be directly used as targets for the robot.

Note that it is possible to modify the robot's base frame in other ways, for example by
changing the location of the ``base_link`` in the RobotModel, or by adding a floating joint
in the RobotModel to represent the mobile platform. However, these methods require
modifying the RobotModel, which can be challenging for some backends. The modifiable
``robot_base_frame`` offered by **COMPAS FAB** provides an easier alternative to the user.