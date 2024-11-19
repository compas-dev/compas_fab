*******************************************************************************
Plan motion
*******************************************************************************

.. note::

   The description on this page is specific to the MoveIt! planner backend,
   see :ref:`Core Concepts: Plan Motion<plan_motion>`
   for general information on plan _plan_motion.


.. currentmodule:: compas_fab.backends

:class:`MoveItPlanner` works with :class:`RosClient` to provides motion planning
capability using the MoveIt! planner backend.
The planner uses an RRTConnect algorithm to create collision-free
:class:`~compas_fab.robots.Trajectory` that connects the start state with the target.

The planner supports
:class:`~compas_fab.robots.FrameTarget`,
:class:`~compas_fab.robots.ConfigurationTarget`, and
:class:`~compas_fab.robots.ConstraintSetTarget` as the planning target.

Available Methods
=================

The :meth:`MoveItPlanner.plan_motion` calculates the motion plan for a given target
and robot cell state. It returns a single solution for the motion planning problem.
The following example shows how to create a starting state, a target, and call the
``plan_motion`` method with the UR5 robot loaded from the running ROS server.

.. literalinclude :: files/05_plan_motion.py
   :language: python

Examples
========

The following example shows the same motion planning problem as above, but with a
collision object added in between the start and end states. This will cause the
planner to find a collision-free path around the obstacle.

.. literalinclude :: files/05_plan_motion_with_obstacle.py
   :language: python

.. currentmodule:: compas_fab.robots

The following example shows planning with an attached tool and an attached workpiece
in the robot cell. The RobotCell is loaded from RobotCellLibrary. The target uses
:class:`TargetMode.WORKPIECE<TargetMode>`, meaning that the robot
will be moved until the OCF of the attached workpiece matches with
:class:`FrameTarget.frame<FrameTarget>`.

.. literalinclude :: files/05_plan_motion_with_attachment.py
   :language: python


:class:`MoveItPlanner` supports custom planning tolerance by setting
:class:`FrameTarget.tolerance_position<FrameTarget>` and
:class:`FrameTarget.tolerance_orientation<FrameTarget>`.
If the target tolerance is not set, the default tolerance for the MoveItPlanner is used:
``DEFAULT_TOLERANCE_ORIENTATION = 0.01``, ``DEFAULT_TOLERANCE_POSITION = 0.001``.

.. literalinclude :: files/05_plan_motion_tolerance.py
   :language: python

The following example shows how to plan a motion with a ConfigurationTarget.
The default planning tolerance for each joint value is
``DEFAULT_TOLERANCE_JOINT = 0.01``.

.. literalinclude :: files/05_plan_motion_configuration.py
   :language: python

See Also
========

- `GetMotionPlan Service in MoveIt Noetic <https://docs.ros.org/en/noetic/api/moveit_msgs/html/srv/GetMotionPlan.html>`_