*******************************************************************************
Forward and inverse kinematics
*******************************************************************************

.. figure:: files/02_forward_and_inverse_kinematics.jpg
    :figclass: figure
    :class: figure-img img-fluid

.. note::

    The following examples use the `ROS <http://www.ros.org/>`_ backend
    and the MoveI! planner for UR5 robots. Before running them, please
    make sure you have the :ref:`ROS backend <ros_backend>` correctly
    configured and the :ref:`UR5 Planner <ros_bundles_list>` started.

Forward kinematics
==================

The forward kinematics function calculates the pose of the robot's end-effector
from joint states (**cartesian space** to **joint space**). This means the
state of each joint in the articulated body of a robot needs to be defined.

Joint states are described in **COMPAS FAB** with the
:class:`compas_fab.robots.Configuration` class.

.. literalinclude :: files/02_forward_kinematics.py
   :language: python

Inverse kinematics
==================

Inverse kinematics is the inverse function of forward kinematics. The
inverse kinematics function calculates the joint states required for the
end-effector to reach a certain target pose (**joint space** to
**cartesian space**).

The following code exemplifies how to calculate this:

.. literalinclude :: files/02_inverse_kinematics.py
   :language: python
