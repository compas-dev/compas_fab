Using Grasshopper to Interact with COMPAS Fab
=============================================

Grasshopper is a node based visual programming language integrated with Rhino,
which allows users to create algorithms and workflows with built-in Grasshopper
components and custom Python scripts. Users can therefore use the "Python 3 Script"
component to write Python code that interacts with COMPAS Fab, enabling them to
create custom workflows for robotic fabrication tasks.

It is important to be aware that Grasshopper manages the execution sequence of
the components on the canvas and automatically detect changes in the data flow
to trigger re-computation. It is the user's responsibility to ensure that the
data flow is set up correctly to avoid illogical computation order,
infinite loops or excessive recomputation.

The examples below focus on creating custom Python scripts with COMPAS Fab
functionality, users can also use the pre-made Grasshopper components
that can be installed via Rhino's package manager. The two approaches can
be combined.

.. toctree::
    :maxdepth: 1
    :glob:

    *
