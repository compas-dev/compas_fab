********************************************************************************
Getting started
********************************************************************************

**COMPAS FAB** is a framework for connecting users with different CAD software
(or the lack thereof in some cases) to different backends that provides robotic
services (e.g. collision detection, simulation, motion planning, etc.).

The following list will guide you through setting up the framework and running
your first example.

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :hidden:
    :glob:

    installation/install_compas_fab
    installation/setup_frontend_rhino
    installation/setup_frontend_rhino_8
    installation/setup_frontend_blender
    installation/setup_frontend_vscode
    installation/setup_frontend_viewer
    installation/setup_backend_no_backend
    installation/setup_backend_ros
    installation/setup_backend_pybullet

#. Step 1: :ref:`install_compas_fab`

#. Step 2: Setup CAD environment to use COMPAS FAB (choose at least one)

   * :ref:`setup_frontend_rhino`

   * :ref:`setup_frontend_rhino_8`

   * :ref:`setup_frontend_blender`

   * :ref:`setup_frontend_vscode`

   * :ref:`setup_frontend_viewer`

#. Step 3: Setup a backend (choose at least one)

   * :ref:`setup_backend_no_backend`

   * :ref:`setup_backend_ros`

   * :ref:`setup_backend_pybullet`

Different front-ends and back-ends have different requirements and support
different planning features. Before starting to work with the framework,
it is best to determine which one is best suited for your needs.


The following table provides a quick overview of the different back-ends
features and requirements:

.. csv-table:: Front-end features and requirements
   :file: frontends/features_and_requirements.csv
   :widths: 20, 20 ,20, 20, 20
   :header-rows: 1

