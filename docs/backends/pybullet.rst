.. _backends_pybullet:

********************************************************************************
Using PyBullet Planner
********************************************************************************

.. highlight:: bash

`PyBullet <https://pybullet.org/>`_ is a Python module extending Bullet, an open
source collision detection and rigid dynamics library written in C++.  PyBullet
was written with the intention of being a "fast and easy to use Python module for
robotics simulation and machine learning."  It also provides bindings for rendering
and visualization, and support for virtual reality headsets.  While PyBullet
is based on a client-server architecture, there is no need to spin up any Docker
containers to run the server.  This, along with its speed, may make PyBullet a
preferable backend for COMPAS_FAB.

PyBullet is able to import Robot Models as URDF packages and provides forward
and inverse kinematics functions. Unlike ROS, the robot models can be loaded
dynamically and multiple robots can be loaded in the same environment. This allows
for the simulation of kinematic tools.

The **COMPAS FAB** PyBullet backend provides the :class:`PyBulletClient` and the
:class:`PyBulletPlanner` classes to interact with PyBullet. The motion planning
functions are implemented by **COMPAS FAB**, building on top of the inverse
kinematics and collision checking functions offered by PyBullet.

PyBullet is not compatible with IronPython. Therefor, it is not possible to use
it with Rhino 7 or its associated Grasshopper.

Next Steps
==========

* :doc:`Tutorial: COMPAS Robots <compas_robots:tutorial>`
* :ref:`Examples: Description models <examples_description_models>`
* :ref:`Examples: PyBullet Backend <examples_pybullet>`
* :ref:`COMPAS FAB API Reference <reference>`
