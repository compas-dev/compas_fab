.. _inverse_kinematics:

*******************************************************************************
Inverse Kinematics
*******************************************************************************

Inverse kinematics is one of the fundamental functions in robotics. It can be
seen as the inverse of the forward kinematics problem. The goal is to find the
joint values of a robot that would result in a given target pose.

Mathematically, if the forward kinematics problem is defined as:

.. math::

    x = f(q)

where :math:`x` is the target (i.e. end effector pose), :math:`q` is the joint
values (angles for revolute joints, displacements for prismatic joints), and
:math:`f` is the forward kinematics function. The inverse kinematics problem
reverses this, and where given a target :math:`x_target`, find :math:`q` such
that:

.. math::

    x_target = f(q)

The inverse kinematics function is used in many applications when the programmer
needs to control the robot's end effector pose. For example, in a pick-and-place
operation, the robot needs to move to a specific location to pick an object and
then move to another location to place it.

Planning functions (such as some Cartesian motion planning algorithms)
are built on top of the inverse kinematics function.

See :ref:`Core Concepts: Targets<target>` for more information on how targets
are defined.

.. _start_configuration:
Starting Configuration
======================

.. _approximation_ik:
Approximation Solver
====================
Approximation method is sensitive to the initial guess (the starting configuration).
It can also be slow to converge near the target.
It may also get stuck in a local minima if the target is not directly reachable
from the starting configuration.
Axis joint limits can also cause the solver to get stuck in a local minima.

Typically a random restart strategy is used to overcome this problem.
If the initial guess does not result in convergence, the solver is restarted with a
different initial guess. When given sufficient number of restarts, the solver is
likely to find a solution, if one exists.

.. _analytical_ik:
Analytical Solver
=================



Finite and Infinite Solutions
=============================
Depending on the solver's behavior, the DOF of the robot and the DOF of the
target, the generator maybe finite or infinite.


Reachability
===========
Axis joint limits
Target too far or too close

Collision
=========
Self collision, including attached objects
Collision with obstacles
See :ref:`collision_check`

Collision Avoidance with null space

Singularities is not a problem for finding a solution, but it can be a problem
during execution.




IK Methods in COMPAS FAB
========================

There are two methods available, which caters to different use cases.

:meth:`PlannerInterface.iter_inverse_kinematics` returns a
generator that yields multiple solutions for the inverse kinematics problem.
Depending on the solver's behavior, the first solution may be close to the
starting configuration (as specified in the robot cell state), while the
subsequent solutions may be further away (refer to planner specific IK
documentation for more details).
The function will attempt to return unique solutions (if supported by
the planner).
This generator mechanism can be useful when the user needs to search through
different solutions to find the one that best suits their application.

:meth:`PlannerInterface.inverse_kinematics` calculates the inverse kinematics
for a given target and robot cell state. It returns a single solution for
the inverse kinematics problem. The returned solution may not may not be
close to the starting configuration (as specified in the robot cell state)
depending on the Planner's behavior.

In many planning backend, the :meth:`PlannerInterface.inverse_kinematics` is
implemented with an automatic caching mechanism that wrapped the
:meth:`MoveItPlanner.iter_inverse_kinematics` method. When the user calls the
:meth:`MoveItPlanner.inverse_kinematics` method multiple times with exactly
the same inputs, the caching mechanism will reuse the previous generator to
return a different solution each time. If the inputs are different it will
create a new generator and return the first solution. If a reused generator
is exhausted, the wrapper will create a new generator hence 'recycling'
the solutions.


Both methods expects a :class:`~compas_fab.robots.RobotCellState` as input,
from which the :class:`~compas_robots.Configuration` of the robot is used
as the :ref:`starting configuration<start_configuration>`.
Attached tools and rigid bodies are also taken into account when calculating
the inverse kinematics (for collision checking and for computing target frame).

A :class:`~compas_fab.robots.Target` object is used to define the target.
Currently, only :class:`~compas_fab.robots.FrameTarget` is supported
(see :ref:`Core Concepts: Frame Target<frame_target>`) by the MoveIt! planner.
Note that the :class:`.target_model<compas_fab.robots.FrameTarget>` attribute
defines how the target is interpreted (see :ref:`target_mode`).

The name of the planning group is expected, which if left empty, will default
to the main planning group (equivalent to
:class:`RobotCell.main_group_name<compas_fab.robots.RobotCell>`).
The default IK solver only works with planning groups that forms a serial chain.
(see :ref:`planning_group`)

Backend Features for Inverse Kinematics
=======================================

.. list-table:: Backend Features for Inverse Kinematics

   :widths: 20 20 20 20 20
   :header-rows: 1

   * - Features
     - MoveIt Planner
     - PyBullet Planner
     - Analytical Kinematics Planner
     - Analytical Pybullet Planner
   * - :meth:`inverse_kinematics`
     - Yes
     - Yes
     - Yes
     - Yes
   * - :meth:`iter_inverse_kinematics`
     - Yes
     - Yes
     - Yes
     - Yes
   * - Solver Type
     - Approximation
     - Approximation
     - Analytical
     - Analytical
   * - Collision Check
     - Yes
     - Yes
     - Yes
     - Yes
   * - :ref:`FrameTarget<frame_target>`
     - Yes
     - Yes
     - Yes
     - Yes
   * - :ref:`PointAxisTarget<point_axis_target>`
     - No
     - Yes
     - No
     - No
