*******************************************************************************
Analytical kinematics
*******************************************************************************

For some applications it is useful to retrieve more than one solution for an
inverse kinematics request, this can be achieved through analytic solvers.
For a general 6-DoF robot (industrial arm with 6 revolute joints), up to eight
possible solutions can be found. (More, if we include joint positions ± 360 degrees).

The resulting eight solutions have an order. That means that if you call IK on
two subsequent frames and compare the 8 configurations of the first frame with
the 8 configurations of the second frame at their respective indices, then these
configurations are "close" to one another. For this reason, for certain use
cases, e.g. for cartesian path planning, it makes sense to keep the order of
solutions. This can be achieved by setting the optional parameter ``keep_order``
to ``True``. The configurations that are in collision or outside joint
boundaries are then not removed from the list of solutions, they are set to
``None``.

We currently support the following robotic arms:

* Offset-Wrist manipulators:
   * UR3, UR3e
   * UR5, UR5e
   * UR10, UR10e

* Spherical-Wrist manipulators:
   * Staubli_TX260L
   * ABB_IRB4600_40_255


Links
=====

* `DH parameters for UR robots <https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics>`_
* `UR kinematics <https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp>`_
* `Lobster : A tool for solving the inverse-kinematics of 6-axis robot arms <https://www.grasshopper3d.com/group/lobster>`_


Inverse kinematics
==================

The inverse kinematics function calculates the joint states required for the
end-effector to reach a certain target pose. 


Here is an example of using ``AnalyticalInverseKinematics``:

.. literalinclude :: files/01_iter_ik_stanalone.py
   :language: python


The above solutions, could however be in collision. In order to check for collisions,
we have to use a `client`. See below the example for using the ``PyBulletClient``

.. literalinclude :: files/02_iter_ik_pybullet.py
   :language: python

Or, alternatively, use the ``AnalyticalPyBulletClient``:

.. literalinclude :: files/03_iter_ik_analytic_pybullet.py
   :language: python


We can also use the ``AnalyticalPyBulletClient`` to calculate a cartesian path:

.. literalinclude :: files/04_cartesian_path_analytic_pybullet.py
   :language: python
