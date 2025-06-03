.. _kinematic_tool_model:

*******************************************************************************
Kinematic Tool Model
*******************************************************************************

Kinematic tools are defined similar to defining a RobotsModel, with a set of
:class:`compas_robots.Link` and :class:`compas_robots.Joint` objects. The Joints
allow the tool to move and change shape, similar to how a robotic arm
articulates. 

The shape (configuration) of the tool is represented using a
:class:`compas_robots.Configuration`, and is defined by setting 
:attr:`ToolState.configuration<compas_fab.robots.ToolState>`.
It can be convenient for the user to define some known tool states, such as
the open and closed states of a gripper, or the extended and retracted states of a
pneumatic cylinder. These states can be defined in the user's code and can 
be used to set the tool's configuration in the :class:`ToolState` object.

The following ROS example show how to set the configuration of a kinematic tool, 
the code for other backends are similar.

.. literalinclude:: ../../backends/ros/files/02_set_robot_cell_state_with_kinematic_tools.py
   :language: python

