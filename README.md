# COMPAS FAB: Robotic Fabrication for COMPAS

[![Github Actions Build Status](https://github.com/compas-dev/compas_fab/workflows/build/badge.svg)](https://github.com/compas-dev/compas_fab/actions)
[![Github Actions Integration Status](https://github.com/compas-dev/compas_fab/workflows/integration/badge.svg)](https://github.com/compas-dev/compas_fab/actions)
[![License](https://img.shields.io/github/license/compas-dev/compas_fab.svg)](https://pypi.python.org/pypi/compas_fab)
[![Conda Downloads](https://img.shields.io/conda/dn/conda-forge/compas_fab)](https://anaconda.org/conda-forge/compas_fab)
[![pip downloads](https://img.shields.io/pypi/dm/compas_fab)](https://pypi.python.org/project/compas_fab)
[![PyPI Package latest release](https://img.shields.io/pypi/v/compas_fab.svg)](https://pypi.python.org/pypi/compas_fab)
[![Anaconda](https://img.shields.io/conda/vn/conda-forge/compas_fab.svg)](https://anaconda.org/conda-forge/compas_fab)
[![Supported implementations](https://img.shields.io/pypi/implementation/compas_fab.svg)](https://pypi.python.org/pypi/compas_fab)
[![DOI](https://zenodo.org/badge/107952684.svg)](https://zenodo.org/badge/latestdoi/107952684)
[![Twitter Follow](https://img.shields.io/twitter/follow/compas_dev?style=social)](https://twitter.com/compas_dev)

*Robotic fabrication package for the COMPAS Framework** that facilitates the
planning and execution of robotic fabrication processes. It provides interfaces
to existing software libraries and tools available in the field of robotics
(e.g. OMPL, ROS) and makes them accessible from within the parametric design
environment. The package builds upon [COMPAS](https://compas.dev/),
an open-source Python-based framework for collaboration and research in
architecture, engineering and digital fabrication.


## Main features

* Multiple backends for simulation and execution (eg. [ROS: Robot Operating System](https://ros.org))
* Planning tools: kinematic solvers, path planning, etc.
* Execution tools: feedback loops, robot control, etc.

**COMPAS FAB** runs on Python 3.x and IronPython 2.7.


## Getting Started

The recommended way to install **COMPAS FAB** is to use [anaconda/conda](https://conda.io/docs/):

    conda install -c conda-forge compas_fab

It can also be installed using `pip`:

    pip install compas_fab


> On Windows, you may need to install [Microsoft Visual C++ 14.0](https://www.scivision.co/python-windows-visual-c++-14-required/).


Once the installation is completed, you can verify your setup.
Start Python from the command prompt and run the following:

    >>> import compas_fab


## First Steps

* [Documentation](https://gramaziokohler.github.io/compas_fab/)
* [COMPAS FAB Examples](https://gramaziokohler.github.io/compas_fab/latest/examples.html)
* [COMPAS FAB API Reference](https://gramaziokohler.github.io/compas_fab/latest/reference.html)
* [COMPAS Tutorials](https://compas.dev/compas/latest/tutorial.html)
* [COMPAS API Reference](https://compas.dev/compas/latest/api.html)


## Questions and feedback

We encourage the use of the [COMPAS framework forum](https://forum.compas-framework.org/)
for questions and discussions.


## Contributing

We love contributions!

Check the [Contributor's Guide](https://github.com/compas-dev/compas_fab/blob/main/CONTRIBUTING.rst)
for more details.


## Releasing this project

Ready to release a new version of **COMPAS FAB**? Here's how to do it:

* We use [semver][https://semver.org/], ie. we bump versions as follows:

  * `patch`: bugfixes.
  * `minor`: backwards-compatible features added.
  * `major`: backwards-incompatible changes.

* Update the `CHANGELOG.md` with all novelty!
* Ready? Release everything in one command:

      invoke release [patch|minor|major]

* Celebrate! 💃

## Credits

This package is maintained by Gramazio Kohler Research [`@gramaziokohler`](https://github.com/gramaziokohler)
and a long list of [contributors](https://github.com/compas-dev/compas_fab/blob/main/AUTHORS.rst).
