.. _backends:

********************************************************************************
Working with backends
********************************************************************************

.. highlight:: bash

One of the driving principles of **COMPAS** framework is to create an ecosystem
and serve as an interface between different front-ends (e.g. CAD software) and
back-ends.


As such, **COMPAS FAB** provides a unified interface to access different robotic platforms
as backends for motion planning.
This chapter highlights the supported backends, their capabilities and the steps to set them up.
The following backends are currently supported:

- **ROS MoveIt!** (OMPL planner implemented by MoveIt, maintained by ROS community, requires a Linux environment, WSL or Docker)
- **PyBullet Planner** (Planner implemented as part of compas_fab, based on the PyBullet physics simulator. Can run natively on Windows, Mac or Linux)
- **Analytical Kinematics Planner** (Planner implemented as part of compas_fab. Limited to selected 6DOF articulated robot kinematics. Can run natively on Windows, Mac or Linux)

Different planners have different capabilities (can plan different types of motions and targets)
and limitations (types of robots kinematics).
For regular users it is important to choose a backend that best fits their needs.
For more advanced users, it is possible to develop new backends or extend existing ones
(see :ref:`backend_architecture`).
The following tables provides a comparison of the different backends.

.. list-table:: Planning capabilities of different backends
    :widths: 25 50 50 50
    :header-rows: 1

    *   - Capabilities
        - ROS MoveIt!
        - PyBullet Planner
        - Analytical Inverse Kinematics Planner
    *   - Forward Kinematics
        - Yes
        - Yes
        - Yes
    *   - Inverse Kinematics
        - Yes (Gradient Based)
        - Yes (Gradient Based)
        - Yes (Analytical)
    *   - Cartesian-Space Motion Planning (Frame Waypoints)
        - Yes
        - Yes
        - Yes (No Collision Detection)
    *   - Cartesian-Space Motion Planning (Point-Axis Waypoints)
        - No
        - Yes
        - No
    *   - Free-Space Motion Planning (Frame Target)
        - Yes
        - Yes
        - No
    *   - Free-Space Motion Planning (Point-Axis Target)
        - No
        - Yes
        - No
    *   - Free-Space Motion Planning (Configuration Target)
        - Yes
        - Yes
        - No

.. list-table:: Running Environment of different backends
    :widths: 25 50 50 50
    :header-rows: 1

    *   -
        - ROS MoveIt!
        - PyBullet Planner
        - Analytical Inverse Kinematics Planner
    *   - Windows
        - Yes (via WSL or Docker)
        - Yes (Already installed with compas_fab)
        - Yes (Pure Python Implementation in compas_fab)
    *   - Linux
        - Yes (Native install or Docker)
        - Yes (Already installed with compas_fab)
        - Yes (Pure Python Implementation in compas_fab)
    *   - Mac
        - Yes (Native install or Docker)
        - Yes (Already installed with compas_fab)
        - Yes (Pure Python Implementation in compas_fab)

.. list-table:: Supported robot kinematics of different backends
    :widths: 25 50 50 50
    :header-rows: 1

    *   -
        - ROS MoveIt!
        - PyBullet Planner
        - Analytical Inverse Kinematics Planner
    *   - Kinematics
        - Supports arbitrary amount of revolute and prismatic joints.
        - Supports arbitrary amount of revolute and prismatic joints.
        - Limited to spherical-wrist and offset-wrist 6DOF articulated robots.
    *   - Pre-installed Robots
        - A selection of industrial robots from the ROS-Industrial repository are pre-installed in the Docker distribution provided by compas_fab.
          See corresponding Dockerfile in `Ros Docker Distribution <https://github.com/gramaziokohler/ros_docker>`_
        - Any robot with a URDF, SRDF and meshes description can be used.
          Robots do not need to be pre-installed, they can be loaded to the backend at runtime.
        - A selection of 6 DOF industrial robots are available as part of the compas_fab package.
          See :ref:`analytical_kinematics` for full list.
    *   - Installing New Robots
        - Require modifying the ROS environment or the Docker container.
        - Custom robots can be loaded to backend at runtime.
        - Custom robots can be added by finding out the robot kinematics and creating subclass of :class:`~compas_fab.backends.OffsetWristKinematics` or :class:`~compas_fab.backends.SphericalWristKinematics`.

Installing backends
===================

Backends can be installed in different ways. Some backends are very simple to
install, while others are very complex.

In order to simplify working with these tools and have a consistent way
to use and test different backends, **COMPAS FAB** provides them as
`Docker containers`_. Docker containers are a way to bundle systems into
isolated, standardized software units with full reproducibility. It greatly
reduces the time it takes to get a backend up and running.

However, all of them can be installed *without* Docker as well. Please refer
to the documentation of the respective tool for standard installation
instructions.

Installing Docker
-----------------

Download and install `Docker Desktop`_:

* `Docker for Windows`_
* `Docker for Mac`_

.. note::

    Make sure you have enabled virtualization in your BIOS.
    Docker will complain if not.

.. note::

    If you're a Windows user, you will need at least Windows 10 Pro.

    After installation, make sure Docker runs in Linux containers mode: right-click
    the docker icon on the tray bar; if there is an option to ``Switch to Linux containers``,
    **select it** and wait for Docker to switch before moving forward.

Working with containers
-----------------------

For Visual Studio Code users, we recommend installing the ``Docker`` extension.


Next steps
==========

Check documentation for your backend of choice:

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :glob:

    backends/*


.. _Docker: https://www.docker.com/
.. _Docker Desktop: https://www.docker.com/get-started
.. _Docker containers: https://www.docker.com/resources/what-container
.. _Docker for Windows: https://hub.docker.com/editions/community/docker-ce-desktop-windows
.. _Docker for Mac: https://hub.docker.com/editions/community/docker-ce-desktop-mac
