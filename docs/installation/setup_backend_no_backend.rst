.. _setup_backend_no_backend:

*******************************************************************************
Working without backend
*******************************************************************************

Without a planning backend, the user can still use the core functionality of
**COMPAS FAB** to create robots and to visualize robots with a CAD environment.
The following functions are still available:

.. currentmodule:: compas_fab

#. Create or load a :class:`robots.RobotCell` instance that includes a
   :class:`compas_robots.RobotModel` instance, :class:`compas_robots.ToolModel`
   instance(s) and :class:`robots.RigidBody` instance(s).

#. Create a :class:`robots.RobotCellState` instance to model the current state,
   including the :class:`compas_robots.Configuration` of the robot,
   :class:`robots.ToolState` and :class:`robots.RigidBodyState` instances.

#. Create a :class:`robots.Target` or :class:`robots.Waypoints` instance and
   serialize them such that they can be planned on another computer with a
   planning backend.

#. Visualize the RobotCell and its associated RobotCellState using one of the
   CAD frontend environments.

#. Compute forward kinematics using the :meth:`compas_robots.RobotModel.forward_kinematics`
   method.

#. Compute forward and inverse kinematics using the :class:`compas_fab.backends.AnalyticalInverseKinematics`
   backend. (This backend does not have collision checking capabilities.)

