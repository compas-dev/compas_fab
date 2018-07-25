.. _examples_coordinate_systems:

********************************************************************************
Coordinate systems
********************************************************************************

.. contents::

The ``Frame`` class can also be used to describe coordinate systems.
Robot targets and positions are located by measurements along the axes of 
coordinate systems. A robot uses several coordinate systems, for example:
* world coordinate system
* base coordinate system // or robot coordinate system??
* tool coordinate system
* work object coordinate system

.. image:: coord_sys.jpg

The world coordinate system (WCS) has its zero point on a fixed position, which 
can be defined by the user's preference. The WCS is useful for handling 
several robots or robots moved by external axes. By default the world coordinate
system coincides with the base coordinate system.

The base coordinate system (BCS) has its zero point in the base of the robot and
is the reference system for the mechanical buildup of the robot. It must be
defined in reference to the WCS. For programming a robot, other coordinate 
systems, like the work object coordinate system are often better choices.

The tool coordinate system (TCS) has its origin at the tip of the tool (Tool 
Center Point: TCP). If the tool is moved, the TCS moves as well. It is dependent
on the BCS.

The work object coordinate system (WOCS) defines the placement of the work piece
or the structure being built in relation to the WCS.

check out:
http://developercenter.robotstudio.com/BlobProxy/manuals/IRC5FlexPendantOpManual/doc210.html#newid-86