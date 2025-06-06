.. _concepts:

*******************************************************************************
Core Concepts
*******************************************************************************

This section explains the core concepts used in the compas_fab library.
Many of these conceptual frameworks are based on the Robot Operating System (ROS) framework,
which is widely used in the field of robotics.

However, the compas_fab library is designed to be a bridge for different
robotic process, and support other planning backends (such as PyBullet Planner),
therefore some of the concepts are different from the ROS convention.
It would be a good idea even for experienced ROS users to read through this section.

.. toctree::
    :maxdepth: 2
    :glob:

    concepts/01_frames/index
    concepts/02_robot_model/index
    concepts/03_tool_model/index
    concepts/04_rigid_body/index
    concepts/05_robot_cell/index
    concepts/06_planning/index
    concepts/07_custom_setup/index

