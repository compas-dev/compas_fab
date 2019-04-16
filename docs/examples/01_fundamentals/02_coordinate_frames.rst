********************************************************************************
Coordinate frames
********************************************************************************

.. currentmodule:: compas_fab.robots

Developers and users of robot drivers, models, and libraries need a shared convention
for coordinate frames in order to better integrate and re-use software components. To
plan robotic fabrication processes, the definition of robotic targets has to follow
the convention of a specific relationship between coordinate frames, for example:

* World coordinate frame (WCF)
* Robot coordinate frame (RCF)
* Tool0 coordinate frame (T0CF)
* Tool coordinate frame (TCF)
* Object coordinate frame (OCF)

.. figure:: 02_coord_frames.jpg
    :figclass: figure
    :class: figure-img img-fluid

    Coordinate frame convention of a robotic setup.

To describe these coordinate frames, the :class:`Frame` class of the **COMPAS**
framework is used.

World coordinate frame (WCF)
============================

The world coordinate frame (WCF) has its origin on a fixed position with
its Z-axis pointing upwards (= map in ROS convention).
The WCF is important for processes that use several robots which share one space,
robots with external axes, and mobile robots. By default, the WCF coincides
with the robot coordinate system (RCF).

Robot coordinate frame (RCF)
============================

The robot coordinate frame (RCF) (= base_link in ROS convention) has its origin
in the base of the robot and is the reference system for the mechanical buildup
of the robot. It must be defined in reference to the fixed coordinate frame WCF.

Tool0 coordinate frame (T0CF)
=============================

The tool0 coordinate frame (T0CF) has its origin at the tip of last link of the robot.
It is dependent on the RCF.

Tool coordinate frame (TCF)
===========================

The tool coordinate frame (TCF) has its origin at the tip of the tool (Tool
Center Point: TCP). It must be defined in reference to the T0CF.

Object coordinate frame (OCF)
=============================

The object coordinate frame (OCF) corresponds to the work object or the built
structure. It defines the location of the work object in relation to the world
coordinate frame (WCF).


Example
==================

Here is a simple example of how to express the frame of an object that is defined
in the world coordinate frame in the robot's own coordinate frame before sending
it as a target pose to the robot.

.. code-block:: python

    from compas_fab.robots import Robot

    from compas.robots import RobotModel
    from compas.robots import Joint
    from compas.robots import Link

    from compas.geometry import Frame
    from compas.geometry import Transformation


    robot_model = RobotModel('ur5',
                joints=[
                    Joint('shoulder_pan_joint', 'revolute', parent='base_link', child='shoulder_link'),
                    Joint('shoulder_lift_joint', 'revolute', parent='shoulder_link', child='upper_arm_link'),
                    Joint('elbow_joint', 'revolute', parent='upper_arm_link', child='forearm_link'),
                    Joint('wrist_1_joint', 'revolute', parent='forearm_link', child='wrist_1_link'),
                    Joint('wrist_2_joint', 'revolute', parent='wrist_1_link', child='wrist_2_link'),
                    Joint('wrist_3_joint', 'revolute', parent='wrist_2_link', child='wrist_3_link'),
                ], links=[
                    Link('base_link'),
                    Link('shoulder_link'),
                    Link('upper_arm_link'),
                    Link('forearm_link'),
                    Link('wrist_1_link'),
                    Link('wrist_2_link'),
                    Link('wrist_3_link'),
                ])

    print("robot model: ", robot_model)

    robot = Robot(robot_model)

    point =  [3.0, 2.0, 1.0]
    xaxis =  [1.0, 0.0, 0.0]
    yaxis =  [0.0, 1.0, 0.0]

    robot_coordinate_frame = Frame(point, xaxis, yaxis)
    robot.set_RCF(robot_coordinate_frame)

    print("robot coordinate frame", robot.get_RCF())

    point =  [6.0, 4.0, 2.0]
    xaxis =  [-1.0, 0.0, 0.0]
    yaxis =  [0.0, -1.0, 0.0]

    frame_WCF = Frame(point, xaxis, yaxis)
    print("frame in WCF", frame_WCF)

    frame_RCF = robot.represent_frame_in_RCF(frame_WCF)
    print("frame in RCF", frame_RCF)

    frame_WCF = robot.represent_frame_in_WCF(frame_RCF)
    print("frame in WCF", frame_WCF)

Links
=====

* `ROS REP-105: Coordinate Frames for Mobile Platforms <http://www.ros.org/reps/rep-0105.html#id15>`_
* `ABB: What is a coordinate system? <http://developercenter.robotstudio.com/BlobProxy/manuals/IRC5FlexPendantOpManual/doc210.html>`_

