.. _trajectory:

*******************************************************************************
Trajectory
*******************************************************************************

While the robot's configuration defines the position of the robot's joints at a given time,
a trajectory is a sequence of configurations that describes the robot's motion over time.
Only discrete trajectories (instead of continuous functions) are supported by **COMPAS FAB**.
It is represented by :class:`compas_fab.robots.JointTrajectory`. The trajectory contains
a list of configurations across time, where each configuration is a
:class:`compas_fab.robots.JointTrajectoryPoint` (which inherits from Configuration).

Very often, trajectories are planned by a motion planner, such as those offered by **COMPAS FAB**.
However it is also possible to create a trajectory manually by specifying a the
joint values.

.. todo:: Add an example of how to create a trajectory manually. Showing the joint_name and type input.

.. _continuity:

Trajectories have two important qualities that are useful in practice (1) they are continuous
and (2) they are collision free.

==========
Continuity
==========

Because of the discrete nature of the trajectory, continuity is
achieved by ensuring that the joint values in the trajectory are close to each other. In another
words, the difference between two consecutive joint values should be small. This is referred to as
'joint jump'. **COMPAS FAB** allows users to specify the maximum joint jump as a constraint
when using the planning backends to plan a trajectory.
See :ref:`plan_motion` and :ref:`plan_cartesian_motion` for more information.

==============
Collision Free
==============

Collision detection is often performed by the motion planner during the motion planning process.
Many of the planners offered by **COMPAS FAB** perform collision detection
at the discrete steps of the trajectory. This quasi-static approach is sufficient for many applications,
but it is important to keep the 'allowable joint jump' of the trajectory small enough to ensure that
the robot does not collide when moving between two configurations.

For applications that requires collision detection with long and thin objects, it is important to
choose a small value for the 'allowable joint jump' because a small rotational movement of a
joint can cause a large sweep with the long object. This can result in a collision that is not
detected by the motion planner.

Some of the motion planners (e.g. :class:`compas_fab.backends.AnalyticalInverseKinematics) does
not perform collision detection at all, so it is the user's responsibility to ensure that the
trajectory is collision free.
