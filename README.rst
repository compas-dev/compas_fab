============================================================
compas_fab: Robotic Fabrication for COMPAS
============================================================

.. start-badges

.. image:: https://readthedocs.org/projects/compas_fab/badge/?style=flat
    :target: https://readthedocs.org/projects/compas_fab
    :alt: Documentation Status

.. image:: https://travis-ci.org/gramaziokohler/compas_fab.svg?branch=master
    :target: https://travis-ci.org/gramaziokohler/compas_fab
    :alt: Travis-CI Build Status

.. image:: https://img.shields.io/pypi/l/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: License

.. image:: https://img.shields.io/pypi/v/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: PyPI Package latest release

.. image:: https://img.shields.io/pypi/wheel/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: PyPI Wheel

.. image:: https://img.shields.io/pypi/pyversions/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: Supported versions

.. image:: https://img.shields.io/pypi/implementation/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: Supported implementations

.. end-badges

**Robotic fabrication package for the COMPAS Framework** that facilitates the planning and execution of robotic fabrication processes.
It provides interfaces to existing software libraries and tools available in the field of robotics (e.g. OMPL, ROS) and makes them
accessible from within the parametric design environment. The package builds upon COMPAS, an open-source Python-based framework for
collaboration and research in architecture, engineering and digital fabrication.


Main features
-------------

* Based on `ROS: Robot Operating System <https://ros.org>`_
* Planning tools: kinematic solvers, path planning, etc.
* Execution tools: feedback loops, robot control, etc.

**compas_fab** runs on Python 2.x, 3.x and IronPython 2.7.


Documentation
-------------

The full documentation, including examples and API reference
is available `here <https://gramaziokohler.github.io/compas_fab/>`_.


Installation
------------



Contributing
------------

Make sure you setup your local development environment correctly:

* Clone the `compas_fab <https://github.com/gramaziokohler/compas_fab>`_ repository.
* Install development dependencies and make the project accessible from Rhino:

::

    pip install -r requirements-dev.txt
    invoke add-to-rhino

**You're ready to start working!**

During development, use tasks on the
command line to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke add-to-rhino``: Make the project accessible from Rhino.
* ``invoke``: Show available tasks.

For more details, check the `Contributor's Guide <CONTRIBUTING.rst>`_.


Releasing this project
----------------------

Ready to release a new version **compas_fab**? Here's how to do it:

* We use `semver <http://semver.org/>`_, i.e. we bump versions as follows:

  * ``patch``: bugfixes.
  * ``minor``: backwards-compatible features added.
  * ``major``: backwards-incompatible changes.

* Update the ``CHANGELOG.rst`` with all novelty!
* Ready? Release everything in one command:

::

    invoke release [patch|minor|major]

* Celebrate! 💃

Credits
-------

This package is maintained by Gramazio Kohler Research `@gramaziokohler <https://github.com/gramaziokohler>`_
