********************************************************************************
Kinematic model
********************************************************************************

.. In ROS, the RobotModel_ and RobotState_ classes are the core classes that
give you access to a robot's kinematics.

In the kinematic model of a robot, the connection of different manipulator
joints is known as link, and the integration of two or more links is called
a joint. This kinematic model can be represented as a tree structure. The tree
describes the kinematic chain, i.e., the connection of robotic links with
joints, and the inter-dependendencies of these links. This tree structure plus
the underlying geometric information can be defined in Unified Robot
Description Format (URDF), which describes any robot (see for example
UR5urdf_). If the robot is mounted on external axes, these links and joints
can be added as well.

.. figure:: 03_robot_links_and_joints.jpg
    :figclass: figure
    :class: figure-img img-fluid

    An industrial robot's links and joints, and the according tree structure
    describing the kinematic model.

.. Actually it would be good to have here a robot on a linear axis...

.. _RobotModel: https://docs.ros.org/kinetic/api/moveit_core/html/classmoveit_1_1core_1_1RobotModel.html
.. _RobotState: https://docs.ros.org/kinetic/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
.. _UR5urdf: https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur5.urdf.xacro

Links
==================
Robot links are solid mechanical elements. Depending on the kinematic model, movement
of certain input links allows the output links to move at various motions.

Joints
==================
The joints are the elements in a robot which helps the links to travel in different
kind of movements. The three major types of joints are:

* **Revolute**: A hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits
* **Prismatic**: A sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits
* **Fixed**: Not really a joint because it cannot move, all degrees of freedom are locked.

.. code-block:: python

    from compas.robots import Joint
    from compas.robots import Link

    link = Link("world")
    joint = Joint("world-joint", parent="world")
    link = Link("j0")
    joint = Joint("j0")
    # TODO: built own robot here

.. The RobotState_ class in ROS contains information about the robot at a snapshot in time, storing vectors of joint positions and optionally velocities and accelerations. The RobotState_ also contains helper functions for setting the arm location based on the end effector location (Cartesian pose) and for computing Cartesian trajectories.

.. figure:: 03_robot_model.jpg
    :figclass: figure
    :class: figure-img img-fluid

    Coordinate frames in each joint of the robot.

Links
=====

* `ROS Wiki: URDF Tutorial <https://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file>`_
* `Create your own URDF file <https://www.codemade.io/create-your-own-urdf-file/>`_

