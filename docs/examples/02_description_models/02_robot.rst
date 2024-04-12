.. _robot_model:

*******************************************************************************
Robot models
*******************************************************************************

Robot models can be programatically defined, as shown in the previous examples,
but in most cases, they are loaded from an existing location.
**COMPAS FAB** supports loading models from local files, from remote Github
repositories as well as from a running ROS instance.

Loading model from disk
=======================

The installation of **COMPAS FAB** includes some robot models which are used
to exemplify loading from disk:

.. literalinclude :: files/02_robot_from_disk.py
   :language: python


Loading model from Github
=========================

Since a large amount of robot models defined in URDF are available on Github,
**COMPAS FAB** provides a specialized loader that follows the conventions
defined by ROS to locate a Robot's model and geometry files.

.. literalinclude :: files/02_robot_from_github.py
   :language: python

Loading model from ROS
======================

.. note::

    The following example uses the `ROS <https://www.ros.org/>`_ backend
    and loads the robot description model from it. Before running it, please
    make sure you have the :ref:`ROS backend <ros_backend>` correctly
    configured and the :ref:`Panda Demo <ros_bundles_list>` started.

In most situations, we will load the robot model directly from a running ROS
instance. The following code exemplifies how to do that.

.. literalinclude :: files/02_robot_from_ros.py
   :language: python

.. note::

    For more details about ROS, go to the :ref:`ROS Examples <ros_examples>`.

Additionally, the ROS loader allows to cache the results locally for faster reloads,
to enable this behavior, pass an argument with the folder where the cache should be stored:

.. literalinclude :: files/02_robot_from_ros_with_cache.py
   :language: python

Visualizing robot models
========================

Once a model is loaded, we can visualize it in our favorite design environment.

**COMPAS** includes the concept of ``scene objects``: classes that assist with the
visualization of datastructures and models, in a way that maintains the data
separated from the specific CAD interfaces, while providing a way to leverage
native performance of the CAD environment.

We use the ``compas_robots`` extension and its ``RobotModelObject`` to visualize
robots easily and efficiently.

The following example illustrates how to load an entire robot model from
ROS and render it in Rhino:

.. literalinclude :: files/03_robot_rhino_from_ros.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Visualize robot from ROS (Rhino) (.PY) <files/03_robot_rhino_from_ros.py>`
* :download:`Visualize robot from ROS (Grasshopper) (.GHX) <files/03_robot_grasshopper_from_ros.ghx>`
* :download:`Visualize robot from Github (Rhino) (.PY) <files/03_robot_rhino.py>`
* :download:`Visualize robot from Github (Blender) (.PY) <files/03_robot_blender.py>`
* :download:`Visualize robot from Github (Grasshopper) (.GHX) <files/03_robot_grasshopper.ghx>`

.. raw:: html

    </div>
    </div>
