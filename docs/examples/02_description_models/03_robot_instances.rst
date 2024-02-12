********************************************************************************
RobotModel Packages
********************************************************************************

.. rst-class:: lead

COMPAS Fab provides several read-to-use robot model packages that can be used for
demonstrating the capabilities of the package. These packages can be accessed from the
`compas_fab.robots.RobotLibrary` class. The robot packages are loaded from local data
files located in the folder `src/compas_fab/data/robot_library`
structure that is compatible with the ROS MoveIt and PyBullet planning backends.

The packages can be found in the `compas_robots.data` module. The following packages are available:
* UR5 (6 DOF offset-wrist robot by Universal Robots)
* ABB IRB 4600-40/2.55 (6 DOF spherical-wrist robot by ABB)
* RFL Robotic Setup (`Robotic Fabrication Laboratory <https://ita.arch.ethz.ch/archteclab/rfl.html>`_)
* ...

Origin of the packages
======================

The robot packages originates from ROS MoveIt Robot Packages located in
`gramaziokohler/ros_docker <https://github.com/gramaziokohler/ros_docker/>`_.
The docker files are *composed up* and extracted over ROS to a local folder structure
that is compatible with the COMPAS Fab package. The convertion script can be found in the
`source code repository <https://github.com/gramaziokohler/compas_fab/scripts/extact_robot_package_from_ros.py>`_.
