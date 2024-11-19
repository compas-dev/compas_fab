.. _backends_ros_moveit:

********************************************************************************
Using ROS MoveIt Planner
********************************************************************************

.. currentmodule:: compas_fab

The ROS MoveIt planner is a wrapper that allows you to use the MoveIt motion
planning framework in a running ROS environment. The ROS and MoveIt environment
must be properly set up and running for this planner to work.

The robot definition used by the planner must be installed in the ROS workspace
and launched together with the MoveIt configuration. Contrary to other backends,
the RobotModel cannot be changed after MoveIt is launched.

The user can only modify the initial robot cell, which is retrieved by calling
:meth:`compas_fab.backends.RosClient.load_robot_cell`. It contains
the :class:`compas_robots.RobotModel`  and :class:`compas_fab.robots.RobotSemantics`
but have no tools and rigid bodies in it.
Users can customize this robot cell by adding tools and rigid bodies as needed.

The typical code snippet to use the MoveIt planner is as follows:

.. code-block:: python

    from compas_fab.backends import RosClient
    from compas_fab.backends import MoveItPlanner
    with RosClient() as client:
        planner = MoveItPlanner(client)
        robot_cell = client.load_robot_cell()

.. note::

    The following examples use the `ROS <https://www.ros.org/>`_ backend.
    Before running them, please make sure you have the :ref:`ROS backend <ros_backend>`
    installed and the :ref:`UR5 Demo <ros_bundles_list>` started.
    It maybe beneficial to :ref:`enable GUI <docker_gui>`.

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :glob:

    ros/*


