.. _scaling_robots:

*******************************************************************************
Scaling robots
*******************************************************************************

Robot models loaded from URDF files — whether from disk, Github, or a running
ROS instance — follow the `ROS REP-0103 <https://www.ros.org/reps/rep-0103.html>`_
convention and store all geometry in **meters**. Many CAD environments (Rhino,
Grasshopper, etc.) work in **millimeters** by default. The
:meth:`~compas_fab.robots.Robot.scale` method bridges this gap.

Scaling a robot model
=====================

The simplest way to scale is to load a robot and call
:meth:`~compas_fab.robots.Robot.scale` with the desired multiplier.

.. literalinclude :: files/04_scaling_robot.py
   :language: python

A few things to keep in mind:

* **Absolute, not cumulative.** :meth:`~compas_fab.robots.Robot.scale` always
  sets the scale to the supplied factor relative to the original URDF geometry.
  Calling ``robot.scale(1000)`` twice leaves the robot at ``1000x``, not
  ``1,000,000x``.

* **Idempotent within the same factor.** Repeating a call with the same value
  is a no-op — useful for defensive code that may be called multiple times.

* **Reversible.** Call ``robot.scale(1)`` at any time to revert to the
  original meter-based dimensions.

* **Affects planning.** Once a scale factor is set, all planning inputs and
  outputs (frame positions, trajectory waypoints, IK targets) are
  automatically converted to and from the scaled units so the ROS/backend
  communication always happens in meters.

Scaling when visualizing
========================

When a scene object (artist) is involved, call :meth:`~compas_fab.robots.Robot.scale`
**after** assigning :attr:`~compas_fab.robots.Robot.scene_object`. This
ensures that both the internal model geometry and the CAD visualization are
updated in a single step.

.. note::

    The following example uses the `ROS <https://www.ros.org/>`_ backend.
    Before running it, make sure you have the :ref:`ROS backend <ros_backend>`
    correctly configured and the :ref:`Panda Demo <ros_bundles_list>` started.

.. literalinclude :: files/04_scaling_robot_from_ros.py
   :language: python

Calling :meth:`~compas_fab.robots.Robot.scale` *before* setting
:attr:`~compas_fab.robots.Robot.scene_object` also works: the factor is stored
internally and automatically applied to the scene object when it is assigned
later. Either order is correct; the post-assignment call is shown here because
it makes the intent explicit.

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Scale robot from library (.PY) <files/04_scaling_robot.py>`
* :download:`Scale robot from ROS (Rhino) (.PY) <files/04_scaling_robot_from_ros.py>`

.. raw:: html

    </div>
    </div>
