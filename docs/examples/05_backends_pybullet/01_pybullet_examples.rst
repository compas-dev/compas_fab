.. _pybullet_examples:

*******************************************************************************
Using PyBullet
*******************************************************************************

First Step
==========

The first step is to connect to PyBullet and verify that the system is working.

Copy and paste the following example into a Python script or REPL.  If, when run,
you see the output ``Connected: True``, then everything is working properly.

.. code-block:: python

    from compas_fab.backends import PyBulletClient
    with PyBulletClient(use_gui=False) as client:
        print('Connected:', client.is_connected)

.. note::

    From the PyBullet user manual:
        The GUI connection will create a new graphical user interface (GUI) with 3D OpenGL
        rendering, within the same process space as PyBullet. On Linux and Windows this GUI
        runs in a separate thread, while on OSX it runs in the same thread due to operating
        system limitations. On Mac OSX you may see a spinning wheel in the OpenGL Window,
        until you run a 'stepSimulation' or other PyBullet command.

Our first example loads the UR5 robot from a URDF and then adds, then removes, a
floor as a collision mesh.  The calls to ``sleep`` are only necessary to prevent the
gui from closing this example too quickly.

.. skip: next

.. literalinclude :: files/01_add_collision_mesh.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Add Collision Mesh (.PY) <files/01_add_collision_mesh.py>`

.. raw:: html

    </div>
    </div>

Adding and removing a collision mesh attached to the end effector link of the
robot is similar.  Again, the calls to ``sleep`` and ``step_simulation`` exist only
to make the GUI rendering smoother.

.. skip: next

.. literalinclude :: files/02_add_attached_collision_mesh.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Add Attached Collision Mesh (.PY) <files/02_add_attached_collision_mesh.py>`

.. raw:: html

    </div>
    </div>

