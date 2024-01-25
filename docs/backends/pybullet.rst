.. _pybullet_backend:

****************
PyBullet
****************

.. highlight:: bash

`PyBullet <https://pybullet.org/>`_ is a Python module extending Bullet, an open
source collision detection and rigid dynamics library written in C++.  PyBullet
was written with the intention of being a "fast and easy to use Python module for
robotics simulation and machine learning."  It also provides bindings for rendering
and visualization, and support for virtual reality headsets.  While PyBullet
is based on a client-server architecture, there is no need to spin up any Docker
containers to run the server.  This, along with its speed, may make PyBullet a
preferable backend for COMPAS_FAB.  However, it, alone, does not provide motion
planning functionality.  PyBullet is also not compatible with IronPython. Hence to use
it with Rhinoceros and Grasshopper it must be invoked through the
:mod:`compas.rpc` module.

Next Steps
==========

* `Tutorial: COMPAS Robots <https://compas.dev/compas/1.17.9/tutorial/robots.html>`__
* :ref:`Examples: Description models <examples_description_models>`
* :ref:`Examples: PyBullet Backend <examples_pybullet>`
* :ref:`COMPAS FAB API Reference <reference>`

..
  TODO: use intersphinx link for compas robots tutorial when new compas sphinx is settled
  Something like this: * :ref:`Tutorial: COMPAS Robots <compas:robots>`
