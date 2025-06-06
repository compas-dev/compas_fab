.. _setup_frontend_rhino_8:

*******************************************************************************
Setup Rhino 8 (CPython)
*******************************************************************************

Setup for Rhino 8
=================

COMPAS FAB is compatible with Rhino 8 and can be installed by Rhino as a
requirement package. Simply add the `r` a Python 3 script component to your Grasshopper
canvas and run the following code to install COMPAS FAB:

.. code-block:: python

    # r: compas_fab
    # venv: compas_fab

    import compas
    print(compas.__file__)
    print(compas.__version__)
    import compas_fab
    print(compas_fab.__file__)
    print(compas_fab.__version__)

This component should be the first component in your Grasshopper canvas
at the left-most side of the connected script. The other components in
your Grasshopper canvas, should contain the `# venv: compas_fab`
comment in the first line of the component to ensure that the correct environment is used.


Setup for Developer
===================

If you are a developer, you may prefer to create a symlink from the editable
compas_fab installation folder in your development environment to the
``compas_fab`` folder in the environment folder of your Rhino 8 CPython installation.

First go to Grasshopper and create a new Python 3 script component.
Create a Python 3 script componen and paste the following code into it:

.. code-block:: python

    # r: compas, compas_robots, roslibpy, scipy
    # venv: compas_fab_dev

    import compas
    print(compas.__file__)
    print(compas.__version__)
    import compas_fab
    print(compas_fab.__file__)
    print(compas_fab.__version__)


This will create a virtual environment in the Rhino 8 environment folder
and install the required packages for compas_fab development. The code
will print out the path of the newly created environment.
It will then produce an error message because ``compas_fab`` module is not found.
The path may look like:

.. code-block:: none

    C:\Users\Username\.rhinocode\py39-rh8\site-envs\compas_fab_dev-Aai3tYqD\compas\__init__.py

Save this Grasshopper file and close Rhino 8.
Create a symlink to the editable installation of COMPAS FAB. On Windows, the command is:

.. code-block:: none

    mklink /J C:\Users\Username\.rhinocode\py39-rh8\site-envs\compas_fab_dev-Aai3tYqD\compas_fab C:\Users\Username\Documents\GitHub\compas_fab\src\compas_fab

Restart Rhino 8 and open the Python 3 script component again.
You should now see the path and version of COMPAS FAB printed out.
