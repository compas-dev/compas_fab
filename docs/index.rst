*******************************************************************************
Robotic Fabrication for COMPAS
*******************************************************************************

.. image:: _images/compas_fab.png
   :class: img-fluid mb-3

.. rst-class:: lead

**Robotic fabrication package for the COMPAS Framework** facilitating the
planning and execution of robotic fabrication processes. It provides interfaces
to existing software libraries and tools available in the field of robotics
(e.g. OMPL, ROS) and makes them accessible from within the parametric design
environment. The package builds upon `COMPAS <https://compas.dev/>`_,
an open-source Python-based framework for collaboration and research in
architecture, engineering and digital fabrication.

Main features
=============

* Built on top of standard model description formats
* Motion planning tools
* Execution tools
* CAD-independent
* Robotic fabrication process modeling
* Multiple robotic backends (e.g. `ROS: Robot Operating System <https://www.ros.org>`_)

**COMPAS FAB** runs on Python 3.x and IronPython 2.7.

.. note::

    **COMPAS FAB** is still under very active development, with new versions
    being released frequently. The interface is not guaranteed to remain stable
    until the package reaches version ``1.0.0``.

Contents
========

.. toctree::
   :maxdepth: 3
   :titlesonly:

   Introduction <self>
   getting_started
   backends
   examples
   api
   contributing
   authors
   changelog
   license

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
