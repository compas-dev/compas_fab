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
See section Conversions
:doc:`COMPAS with Rhino <compas:userguide>`
 for more information.

The workflow of using the native CAD geometry as input are similar across different
CAD environments. For example, in Rhino, the syntax looks like this:

.. code-block:: python

    import Rhino.Geometry
    import compas_rhino.conversions

    point = Rhino.Geometry.Point3d(...)
    point = compas_rhino.conversions.point_to_compas(point)

    line = Rhino.Geometry.Line(...)
    line = compas_rhino.conversions.line_to_compas(line)

    plane = Rhino.Geometry.Plane(...)
    plane = compas_rhino.conversions.plane_to_compas(plane)

The workflow of visualizing the robot cell and planned motions are also similar across
different CAD environments:

.. code-block:: python

    from compas_fab.robots import RobotCellLibrary
    from compas.scene import Scene

    # Load the robot model
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    # Create a scene object for visualization
    scene = Scene()
    scene_object = scene.add(robot_cell)

    # Visualize the robot in the COMPAS Viewer or other CAD environment
    native_geometry = scene_object.draw(robot_cell_state)

Specific examples of how to work with different CAD environments are provided in the
following sections.

Rhino
=====

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/rhino/index

Grasshopper
==================

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/ghpython/index

Blender
==================
.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/blender/index

COMPAS Viewer
==================

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    frontends/viewer/index

