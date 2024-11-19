.. _robot_cell:

********************************************************************************
Robot Cell
********************************************************************************

Robot cell is a container for models of robot, tools and other objects that are
present in a robotic setup. It is a container that holds a single RobotModel, a
dictionary of tools (ToolModel) and a dictionary of rigid bodies (RigidBody).
It is the fundamental building block used in simulation, visualization and
motion planning.

A RobotCell object contains the non-changing information about the robot cell,
such as the robot model, tool models and rigid bodies. Very often these objects
are time-consuming to create and are therefore not modified frequently.

On the other hand, the modifiable information about the robot cell is stored in
a RobotCellState object. Each model in the robot cell have a corresponding state
object in the RobotCellState object. The following table shows the mapping
between the model objects and their corresponding state objects.

.. list-table:: Title
   :widths: 50 50
   :header-rows: 1

   * - Model Object
     - State Object(s)
   * - RobotModel
     - Configuration,\n robot_base_frame
   * - ToolModel
     - ToolState
   * - RigidBody
     - RigidBodyState

Throughout a fabrication process, the robot cell often undergo changes in its
state. For example, the robot may move to a different configuration, tools can
be attached or detached, and rigid bodies can be moved around. These changes
are reflected (modeled) by modifying the corresponding state object.

The RobotCellState object is lightweight and can be easily copied and modified.
Many RobotCellState objects can be created and stored to represent a sequence
of states during a fabrication process. The RobotCellState objects are used
as input when planning motion to specify the starting condition and they are also
used when visualizing the robot cell.


Creating a Robot Cell
=====================
It is typical for the user to create a custom RobotCell for their specific
application. For example a pick and place application may require a gripper
tool and the workpieces to be manipulated. A 3D printing process may require
a 3D printing tool, a build platform and segments of the printed object modelled
as rigid bodies.

Once the RobotCell is created, the user can store it in a file for later use.
It is especially useful to store the RobotCell for visualizing planned motions.

Add a RobotModel
----------------
Compas_fab only supports one robot model in a RobotCell.

Add a robot from URDF packages


Add a robot from a running ROS instance (specific to using with ROS backend)

Add ToolModel(s)
----------------

Add a gripper


Add ToolState(s)
----------------

Note that the attachment relationship between the tool and the robot is defined
in the ToolState object.


Add RigidBody(s)
----------------

Add a robotic dress pack

Add a list of bricks

Add floor and walls to the robot cell

Add RigidBodyState(s)
----------------

The position of the bricks and the attachment relationship of the attached
objects are defined in the RigidBodyState object.

Storing and Loading a Robot Cell
===============================

Serializing a RobotCell


.. _robot_cell_state:

Robot Cell State
================

A RobotCellState must match the RobotCell it is associated with ...


The RobotCell and RobotCellState objects are used together for visualization and
planning.

RobotCellState is used in all the planning functions to describe the state of
the robot cell at the beginning of the planning process. It is used to communicate
the state of everything in the robot cell to the planner, such as the robot's
starting configuration, the state of the tools and the position of the rigid bodies.


********************************************************************************
Robot Cell Library
********************************************************************************

.. rst-class:: lead

``COMPAS FAB`` provides several ready-to-use robot packages that can be used for
demonstrating the capabilities of the package. These packages can be accessed from the
:class:`compas_fab.robots.RobotLibrary`. The robot packages are loaded from local data
files. They contain
the `robot.model`, `robot.semantics` and meshes associated with the robot links.


The following packages are available:


.. list-table:: Robot Packages
    :widths: 25 25 50 25
    :header-rows: 1

    * - Model
      - Brand
      - Description
      - Location
    * - UR5
      - Universal Robots
      - 6 DOF offset-wrist robot
      - :class:`~compas_fab.robots.RobotLibrary.ur5`
    * - UR10e
      - Universal Robots
      - 6 DOF offset-wrist robot
      - :class:`~compas_fab.robots.RobotLibrary.ur10e`
    * - IRB 4600-40/2.55
      - ABB
      - 6 DOF spherical-wrist robot
      - :class:`~compas_fab.robots.RobotLibrary.abb_irb4600_40_255`
    * - RFL
      - ETH Arch_Tec_Lab
      - Multi-​robotic system with 2 gantry and 4 robotic arms
      - :class:`~compas_fab.robots.RobotLibrary.rfl`


Description of the RFL Robotic Setup (`Robotic Fabrication Laboratory <https://ita.arch.ethz.ch/archteclab/rfl.html>`_)


Origin of the packages
----------------------

The robot packages originates from ROS MoveIt Robot Packages located in
`gramaziokohler/ros_docker <https://github.com/gramaziokohler/ros_docker/>`_.
The docker files are *composed up* and extracted over ROS to a local folder structure
that is compatible with the COMPAS Fab package. The convertion script can be found in the
source code repository folder `scripts/extact_robot_package_from_ros.py`.

