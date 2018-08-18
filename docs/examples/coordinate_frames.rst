.. _examples_coordinate_systems:

********************************************************************************
Coordinate frames
********************************************************************************

.. contents::

Developers and users of robot drivers, models, and libraries need a shared convention
for coordinate frames in order to better integrate and re-use software components. To
plan robotic fabrication processes, the definition of robotic targets has to follow
the convention of a specific relationship between coordinate frames, for example:

* World coordinate frame (WCF)
* Robot coordinate frame (RCF)
* Tool0 coordinate frame (T0CF)
* Tool coordinate frame (TCF)
* Object coordinate frame (OCF)

.. image:: coord_sys.jpg

To describe these coordinate frames, the ``Frame`` class of the compas libray is used.

World coordinate frame (WCF)
============================

The world coordinate frame (WCF) has its origin on a fixed position with
its Z-axis pointing upwards (= map in ROS convention). The WCF is important for processes
that use several robots which share one space, robots with external axes, and mobile robots.
By default, the WCF coincides with the robot coordinate system (RCF).

Robot coordinate frame (RCF)
============================

The robot coordinate frame (RCF) (= base_link in ROS convention) has its origin
in the base of the robot and is the reference system for the mechanical buildup of the robot.
It must be defined in reference to the fixed coordinate frame WCF.

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

Links:
http://www.ros.org/reps/rep-0105.html#id15
http://developercenter.robotstudio.com/BlobProxy/manuals/IRC5FlexPendantOpManual/doc210.html

