*******************************************************************************
Full examples
*******************************************************************************

.. note::

    The following examples use the `ROS <http://www.ros.org/>`_ backend
    and the MoveI! planner with the Franka Emika Panda robots.
    Before running them, please make sure you have the :ref:`ROS backend <ros_backend>`
    correctly configured and the :ref:`Panda Demo <ros_bundles_list>` started.

The following lists the 4 main features of the integrated motion planning framework:
forward and inverse kinematics, cartesian and kinematic planning including loading
the entire robot model over ROS, and a complete Grasshopper canvas as a playground
for these features.

Forward Kinematics
=====================

.. literalinclude :: files/02_forward_kinematics_ros_loader.py
   :language: python

Inverse Kinematics
=====================

.. literalinclude :: files/02_inverse_kinematics_ros_loader.py
   :language: python

Plan cartesian motion
=====================

.. literalinclude :: files/04_plan_cartesian_motion_ros_loader.py
   :language: python

Plan motion
===========

.. literalinclude :: files/04_plan_motion_ros_loader.py
   :language: python

Grasshopper playground
======================

The following Grasshopper canvas demonstrate all these features combined in
a single document, as a reference of how these features could be integrated:


.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Download</div>

* :download:`Robot playground (Grasshopper) (.GHX) <files/robot-playground.ghx>`

.. raw:: html

    </div>
    </div>

