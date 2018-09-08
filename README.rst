============================================================
compas_fab: Robotic Fabrication for COMPAS
============================================================

.. start-badges

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
accessible from within the parametric design environment. The package builds upon `COMPAS <https://compas-dev.github.io/>`_,
an open-source Python-based framework for collaboration and research in architecture, engineering and digital fabrication.


Main features
-------------

* Multiple backends for simulation and execution (e.g. `ROS: Robot Operating System <https://ros.org>`_)
* Planning tools: kinematic solvers, path planning, etc.
* Execution tools: feedback loops, robot control, etc.

**compas_fab** runs on Python 2.x, 3.x and IronPython 2.7.


Getting Started
---------------

The recommended way to install **compas_fab** is to use `Anaconda/conda <https://conda.io/docs/>`_:

::

    conda config --add channels conda-forge
    conda install COMPAS
    pip install compas_fab


Once installed, you can verify your setup. Start Python from the command prompt and run the following:

.. code-block:: python

    >>> import compas_fab


First Steps
-----------

* `Documentation <https://gramaziokohler.github.io/compas_fab/>`_
* `Examples <https://gramaziokohler.github.io/compas_fab/latest/examples.html>`_
* `API Reference <https://gramaziokohler.github.io/compas_fab/latest/reference.html>`_
* `COMPAS Examples <https://compas-dev.github.io/main/examples.html>`_
* `COMPAS Tutorials <https://compas-dev.github.io/main/tutorial.html>`_
* `COMPAS API Reference <https://compas-dev.github.io/main/api.html>`_


Questions and feedback
----------------------

We encourage the use of the `COMPAS framework forum <https://forum.compas-framework.org/>`_
for questions and discussions.


Contributing
------------

We love contributions!

Check the `Contributor's Guide <https://github.com/gramaziokohler/compas_fab/blob/master/CONTRIBUTING.rst>`_
for more details.


Releasing this project
----------------------

Ready to release a new version of **compas_fab**? Here's how to do it:

* We use `semver <https://semver.org/>`_, i.e. we bump versions as follows:

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
and a long list of `contributors <https://github.com/gramaziokohler/compas_fab/blob/master/AUTHORS.rst>`_
