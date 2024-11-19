*******************************************************************************
Plan Cartesian Motion
*******************************************************************************

.. note::

   The description on this page is specific to the MoveIt! planner backend,
   see :ref:`Core Concepts: Plan Cartesian Motion<plan_cartesian_motion>`
   for general information on plan Cartesian motion.

.. currentmodule:: compas_fab.backends

:class:`MoveItPlanner` works with :class:`RosClient` to provides Cartesian
motion planning capability using the MoveIt! planner backend.

.. currentmodule:: compas_fab.robots

The planner create collision-free :class:`Trajectory`
that begins at the start state and passes through a number of waypoints.
The planner supports only :class:`FrameWaypoints`, which is defined with a list of
:class:`compas.geometry.Frame` objects. The planner will move the robot such that
the PCF, OCF, or TCP (depending on the :class:`TargetMode`) passes through the
frames with linear segments. See :ref:`frame_waypoints` for more information.

The planner can also interpolate through frames with different orientations.
However, the planner can only guarantee that the center point of the PCF is
moving along a straight line. In another words, if the target mode is
:class:`TargetMode.TOOL<TargetMode>` or :class:`TargetMode.WORKPIECE<TargetMode>`,
the TCF or OCF will not move along a straight line during rotation.

Available Methods
=================

.. currentmodule:: compas_fab.backends
The :meth:`MoveItPlanner.plan_cartesian_motion` calculates the motion plan for a given target
and robot cell state. It returns a single solution for the motion planning problem.
The following example shows how to call the ``plan_cartesian_motion`` method with
the UR5 robot loaded from the running ROS server. The first segment of the trajectory
includes a reorientation while the second segment only includes translation.

.. literalinclude :: files/06_cartesian_motion.py
   :language: python

Examples
========

The following example uses Waypoint with TargetMode.WORKPIECE, meaning that the
attached workpiece will be moved such that its OCF passes through the waypoints.

.. literalinclude :: files/06_cartesian_motion_target_mode.py
   :language: python

The following example shows the effect of ``max_step`` on the trajectory. The
``max_step`` parameter is used to limit the maximum distance between two waypoints.

.. literalinclude :: files/06_cartesian_motion_step_size.py
   :language: python

.. Examples for turning off collision checking.