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
    configured and the :ref:`UR5 Demo <ros_bundles_list>` started.

Forward kinematics
==================

The forward kinematics function calculates the pose of the robot's end-effector
from joint states (**cartesian space** to **joint space**). This means the
state of each joint in the articulated body of a robot needs to be defined.

Joint states are described in **COMPAS FAB** with the
:class:`compas_fab.robots.Configuration` class.

The simplest way to calculate forward kinematics is based on the properties defined
by the robot model and does not require ROS to be running:

.. literalinclude :: files/02_forward_kinematics.py
   :language: python

Additionally, if the :class:`compas_fab.robots.Robot` is assigned a ``client``, it
will try to use it to resolve forward kinematics. The following example shows the same
solutions but calculated by ROS:

.. literalinclude :: files/02_forward_kinematics_ros.py
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
