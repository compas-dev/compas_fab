*******************************************************************************
Robot Cells in ROS
*******************************************************************************

Once ROS is running and MoveIt! planner has started with a robot, we can
start interacting with the robot model.

Load robot cell from ROS
========================

Load the initial robot cell from ROS using
:meth:`compas_fab.backends.RosClient.load_robot_cell`.
It will return a RobotCell object that contains the RobotModel and RobotSemantics
but without any ToolModel or RigidBody objects. This method is unique to the RosClient
because the RobotModel loaded in the backend cannot be modified.

This function allows you to load the running robot cell from ROS and be sure that
the RobotModel and RobotSemantics in the RobotCell is the same as the one in the backend.

.. literalinclude :: files/02_load_robot_cell.py
   :language: python

Customize the robot cell
========================

Users can customize the RobotCell by adding :class:`compas_robots.ToolModel` and
:class:`compas_fab.robots.RigidBody` objects to the robot cell.
After modifying the RobotCell, :meth:`compas_fab.backends.MoveItPlanner.set_robot_cell`
must be called to send the robot cell to the backend.

.. literalinclude :: files/02_set_robot_cell.py
   :language: python

Visualize the robot cell in RViz
===============================

If GUI is enabled, the robot cell will be visualized in RViz after calling
:meth:`compas_fab.backends.MoveItPlanner.set_robot_cell`. However, the position
of the objects may not be in a meaningful position. Users can pass a
:class:`compas_fab.robots.RobotCellState` object to the method, for RViz to
display the robot cell in a specific state.

The following example shows the effect of attaching a tool, attaching a workpiece
and detaching them. Note that the `ToolState.attachment_frame` attribute take effect
when the tool is attached to the robot and the `ToolState.frame` attribute take effect
when the tool is detached from the robot. The same applies to the `RigidBodyState`.

Note that the example make use of `MoveItPlanner.set_robot_cell_state` to set the
robot cell state in the backend. This method is generally not needed to be called
by the user, as it is called internally by the `MoveItPlanner` when a planning
function is called. It is called here only to update the visualization in RViz.

.. literalinclude :: files/02_set_robot_cell_state_attach_objects.py
   :language: python

The following example shows the effect of changing the robot configuration,
which causes the attached tool to move with the robot.

.. literalinclude :: files/02_set_robot_cell_state_with_kinematic_tools.py
   :language: python


Serializing the robot cell
========================

Users can choose to serialize their customized robot cell, which can be used
in another session with the same backend that have been initialized with the
same MoveIt configuration. For example, if the RobotCell comes from a ROS
with a UR5 robot, it will not work with a ROS with a Panda robot. The user
should not modify the RobotModel and RobotSemantics in the serialized file.

All tools and rigid bodies are serialized with the RobotCell. The user can
simply start a new session with the same backend and load the serialized
RobotCell to continue working with the same set of tools and rigid bodies.

.. todo
.. literalinclude :: files/02_serialize_robot_cell.py
   :language: python


