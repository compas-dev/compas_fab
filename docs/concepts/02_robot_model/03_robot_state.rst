.. _robot_state:

*******************************************************************************
Robot State
*******************************************************************************

While the robot model defines the intrinsic properties of the robot, the robot state
defines the extrinsic (changeable) properties of the robot. In **COMPAS FAB**, the
robot's state consist of two components, the robot's joint configuration (represented
by a :class:`compas.robots.Configuration`) and the robot's base frame (represented by a
:class:`compas.geometry.Frame`).

The robot's configuration is a set of joint values that define the positions
of its kinematic joints. The robot's base frame, also referred to as the Robot
Coordinate Frame (RCF), is the frame that defines the robot's position relative
to the world coordinate frame.

Both of these properties are contained by the :class:`compas_fab.robots.RobotCellState`,
which will be :ref:`introduced later <robot_cell_state>`.

.. _configuration:

Robot Configuration
===================

Full Configuration

Group Configuration


Robot Coordinate Frame
======================

The robot coordinate frame (RCF) refers to the location of the robot in the world
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

Another example is when the robot is mounted on a mobile platform, but such kinematics is
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

.. _trajectory:

Trajectory
==========

While the robot's configuration defines the position of the robot's joints at a given time,
a trajectory is a sequence of configurations that describes the robot's motion over time.
The trajectory used by **COMPAS FAB** is discrete, meaning that it is a list of configurations
instead of a continuous function that describes the robot's motion.

Very often, trajectories are planned by a motion planner, such as those offered by **COMPAS FAB**.
However it is also possible to create a trajectory manually by specifying a the
joint values.

.. todo:: Add an example of how to create a trajectory manually. Showing the joint_name and type input.

.. _continuity:

Trajectories have two important qualities that are useful in practice (1) they are continuous
and (2) they are collision free. Because of the discrete nature of the trajectory, continuity is
achieved by ensuring that the joint values in the trajectory are close to each other. In another
words, the difference between two consecutive joint values should be small. This is referred to as
'joint jump'. **COMPAS FAB** allows users to specify the maximum joint jump as a constraint
when using the planning backends to plan a trajectory.
See :ref:`plan_motion` and :ref:`plan_cartesian_motion` for more information.

Collision detection is often performed by the motion planner during the motion planning process.
At the moment, all the default planners offered by **COMPAS FAB** perform collision detection only
at the discrete steps of the trajectory. This quasi-static approach is sufficient for many applications,
but it is important to keep the 'allowable joint jump' of the trajectory small enough to ensure that
the robot does not collide when moving between two configurations.

For applications that requires collision detection with long and thin objects, it is important to
choose a small value for the 'allowable joint jump' because a small rotational movement of a
joint can cause a large sweep with the long object. This can result in a collision that is not
detected by the motion planner.

