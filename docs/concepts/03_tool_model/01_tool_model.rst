.. _tool_model:

*******************************************************************************
Tool Model
*******************************************************************************

Robots are often equipped with tools to perform specific tasks. However, since
the tools are not part of the robot and are sometimes detachable, they are
represented as a separate entity from the RobotModel. A robotic tool can be
static (geometrically fixed), or kinematic (with moving parts). Examples of
static tools include welding torches, paste extruders and drawing pens, these
tools only have a single geometric body. Examples of kinematic tools include
robotic grippers, and robotic hands, these tools have moving parts and can
change their shape.

In order to model both types of tools, **COMPAS FAB** uses the
:class:`compas_robots.ToolModel` class, which is a subclass of
:class:`compas_robots.RobotModel`. This allow us to define the tool's geometry
as `Links` and model their kinematics using `Joints`. Similar to the RobotModel,
a kinematic ToolModel requires a :class:`compas_robots.Configuration` to represent
the state of the tool, such as a opened or closed gripper.

The :class:`ToolLibrary` class provides a collection of ready-made ToolModels
for demonstration purposes, which are used throughout the example files.
They can be used for learning and testing purposes. When you are ready to work with
your own tool, you can :ref:`create your own ToolModel<custom_tool>`
in a similar way as you would create a RobotModel.

Tool Coordinate Frames
======================

A ToolModel is defined in its own coordinate frame, called the Tool Base Coordinate
Frame (TBCF). It is the conceptual origin of the tool where the geometry of the tool
is defined and also where the tool is attached to the robot.

On the other hand, a tool is often used to interact with the environment or workpieces.
Very often, there is a point of interest at the business end of the tool, such as the tip of a
welding torch or the nozzle of an extruder. It is common to program the robot, such that this
point of interest reaches a target pose or passes through a target trajectory. By convention,
this point of interest is called the tool tip or Tool Center Point (TCP). In **COMPAS FAB**,
we define a frame to represent the TCP, called the Tool Coordinate Frame (TCF).
It's is defined relative to the TBCF and its transformation is defined in
:class:`ToolModel.frame<ToolModel>`.

The TCF is one of the three target reference that can be used as to create
:class:`compas_fab.robots.Target` for inverse kinematics calculation and motion planning.

