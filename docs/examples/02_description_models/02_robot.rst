*******************************************************************************
Robot
*******************************************************************************

Visualizing robot models
========================

**COMPAS** includes the concept of `artists`: classes that assist with the
visualization of datastructures and models, in a way that maintains the data
separated from the specific CAD interfaces, while providing a way to leverage
native performance of the CAD environment.

In the main library there are artists for various datastructures (meshes,
networks, etc), and **COMPAS FAB** adds a ``RobotArtist`` to them.
Robot artists allow visualizing robot models easily and efficiently.

The following example illustrates how to load an entire robot model from
an open source repository and render it in Rhino:

.. literalinclude :: files/02_robot_artist_rhino.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Robot artist (Rhino) (.PY) <files/02_robot_artist_rhino.py>`
* :download:`Robot artist (Blender) (.PY) <files/02_robot_artist_blender.py>`
* :download:`Robot artist (Grasshopper) (.GHX) <files/02_robot_artist_grasshopper.ghx>`

.. raw:: html

    </div>
    </div>
