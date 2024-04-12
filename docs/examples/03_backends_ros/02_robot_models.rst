*******************************************************************************
Robots in ROS
*******************************************************************************

.. note::

    The following examples use the `ROS <https://www.ros.org/>`_ backend
    and the MoveIt! planner for UR5 robots. Before running them, please
    make sure you have the :ref:`ROS backend <ros_backend>` correctly
    configured and the :ref:`UR5 Demo <ros_bundles_list>` started.

Once ROS is running and MoveIt! planner has started with a robot, we can
start interacting with the robot model. There are two basic ways to work
with a robot model that is loaded in ROS.

Load model from ROS
====================

This is the easiest and preferable way to load a robot model:
request the full model to be loaded from ROS.

.. literalinclude :: files/02_robot_model.py
   :language: python
