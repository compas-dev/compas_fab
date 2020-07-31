*******************************************************************************
Forward and inverse kinematics
*******************************************************************************

Forward kinematics
==================

As with ROS, the forward kinematics function calculates the pose of the robot's
end-effector from joint states. This means the state of each joint in the
articulated body of a robot needs to be defined.

Joint states are described in **COMPAS FAB** with the
:class:`compas_fab.robots.Configuration` class.

Below we demonstrate calculating the forward kinematics for a UR5 robot.
(Note: Since the PyBullet server has the capacity to load multiple robots,
which robot the forward kinematics are being calculated for must be specified.)

.. literalinclude :: files/02_forward_kinematics.py
   :language: python


Inverse kinematics
==================

Inverse kinematics is the inverse function of forward kinematics. The
inverse kinematics function calculates the joint states required for the
end-effector to reach a certain target pose.

Here is an example of such a calculation using PyBullet:

.. literalinclude :: files/02_inverse_kinematics.py
   :language: python
