.. _pybullet_backend:

****************
PyBullet
****************

.. highlight:: bash

`PyBullet <https://pybullet.org/>`_ is a Python module extending Bullet, an open
source collision detection and rigid dynamics library written in C++.  PyBullet
was written with the intention of being a "fast and easy to use Python module for
robotics simulation and machine learning."  It also provides bindings for rendering
and visualization, and support for virtual reality headsets including HTC Vive and
Oculus Rift.  While PyBullet is based on a client-server architecture, there is no
need to spin up any Docker containers to run the server.  This, along with its speed,
may make PyBullet a preferable backend for COMPAS_FAB.  However, it, alone, does not
provide motion planning functionality.  PyBullet is also not compatible with IronPython.
Hence to use it with Rhinoceros and Grasshopper it must be invoked through `compas.rpc`
(see `COMPAS RPC <https://compas-dev.github.io/main/api/compas.rpc.html>`_).
