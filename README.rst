============================================================
COMPAS FAB: Robotic Fabrication for COMPAS
============================================================

.. start-badges

.. image:: https://github.com/compas-dev/compas_fab/workflows/build/badge.svg
    :target: https://github.com/compas-dev/compas_fab/actions
    :alt: Github Actions Build Status

.. image:: https://github.com/compas-dev/compas_fab/workflows/integration/badge.svg
    :target: https://github.com/compas-dev/compas_fab/actions
    :alt: Github Actions Integration Status

.. image:: https://img.shields.io/github/license/compas-dev/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: License

.. image:: https://img.shields.io/pypi/v/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: PyPI Package latest release

.. image:: https://img.shields.io/conda/vn/conda-forge/compas_fab.svg
    :target: https://anaconda.org/conda-forge/compas_fab

.. image:: https://img.shields.io/pypi/implementation/compas_fab.svg
    :target: https://pypi.python.org/pypi/compas_fab
    :alt: Supported implementations

.. image:: https://zenodo.org/badge/107952684.svg
   :target: https://zenodo.org/badge/latestdoi/107952684

.. end-badges

**Robotic fabrication package for the COMPAS Framework** that facilitates the
planning and execution of robotic fabrication processes. It provides interfaces
to existing software libraries and tools available in the field of robotics
(e.g. OMPL, ROS) and makes them accessible from within the parametric design
environment. The package builds upon `COMPAS <https://compas.dev/>`_,
an open-source Python-based framework for collaboration and research in
architecture, engineering and digital fabrication.


Main features
-------------

* Multiple backends for simulation and execution (e.g. `ROS: Robot Operating System <https://ros.org>`_)
* Planning tools: kinematic solvers, path planning, etc.
* Execution tools: feedback loops, robot control, etc.

**COMPAS FAB** runs on Python 3.x and IronPython 2.7.


Getting Started
---------------

The recommended way to install **COMPAS FAB** is to use `Anaconda/conda <https://conda.io/docs/>`_:

::

    conda install -c conda-forge compas_fab

But it can also be installed using ``pip``:

::

    pip install compas_fab


.. note::

    On Windows, you may need to install
    `Microsoft Visual C++ 14.0 <https://www.scivision.co/python-windows-visual-c++-14-required/>`_.


Once the installation is completed, you can verify your setup.
Start Python from the command prompt and run the following:

::

    >>> import compas_fab


First Steps
-----------

* `Documentation <https://gramaziokohler.github.io/compas_fab/>`_
* `COMPAS FAB Examples <https://gramaziokohler.github.io/compas_fab/latest/examples.html>`_
* `COMPAS FAB API Reference <https://gramaziokohler.github.io/compas_fab/latest/reference.html>`_
* `COMPAS Tutorials <https://compas.dev/compas/tutorial.html>`_
* `COMPAS API Reference <https://compas.dev/compas/api.html>`_


Questions and feedback
----------------------

We encourage the use of the `COMPAS framework forum <https://forum.compas-framework.org/>`_
for questions and discussions.


Contributing
------------

We love contributions!

Check the `Contributor's Guide <https://github.com/compas-dev/compas_fab/blob/master/CONTRIBUTING.rst>`_
for more details.


Releasing this project
----------------------

Ready to release a new version of **COMPAS FAB**? Here's how to do it:

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
and a long list of `contributors <https://github.com/compas-dev/compas_fab/blob/master/AUTHORS.rst>`_
