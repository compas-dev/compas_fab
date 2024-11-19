.. _frontends:

********************************************************************************
Working with CAD Environments
********************************************************************************

.. highlight:: bash

.. currentmodule:: compas_fab

**COMPAS FAB** allows users to work with their favorite CAD software to create and
visualize robot cells, define motion targets and visualize the results of planned
motions.

Using the CAD conversion in **COMPAS FRAMEWORK**, native CAD data can be converted
to COMPAS data format, and vice versa. This allows users to model custom robot cells,
tools, and define targets for motion planning in their choice of CAD software.

The workflow of using the native CAD geometry as input are similar across different
CAD environments:

.. TODO: Add generic example

The workflow of visualizing the robot cell and planned motions are also similar across
different CAD environments:

.. TODO: Add generic example

Specific examples of how to work with different CAD environments are provided in the
following sections.

Rhino
=====

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/rhino

Grasshopper
===========

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/ghpython

Blender
=======

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/blender

viewer
======

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/viewer

