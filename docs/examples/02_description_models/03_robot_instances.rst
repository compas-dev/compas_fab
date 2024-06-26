********************************************************************************
RobotModel Packages
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
======================

The robot packages originates from ROS MoveIt Robot Packages located in
`gramaziokohler/ros_docker <https://github.com/gramaziokohler/ros_docker/>`_.
The docker files are *composed up* and extracted over ROS to a local folder structure
that is compatible with the COMPAS Fab package. The convertion script can be found in the
source code repository folder `scripts/extact_robot_package_from_ros.py`.
