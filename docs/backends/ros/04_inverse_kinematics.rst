.. _ros_inverse_kinematics:

*******************************************************************************
Inverse Kinematics
*******************************************************************************

.. note::

   The description on this page is specific to the MoveIt! planner backend,
   see :ref:`Core Concepts: Inverse Kinematics<inverse_kinematics>`
   for general information on inverse kinematics.

.. currentmodule:: compas_fab.backends

:class:`MoveItPlanner` works with :class:`RosClient` to provides inverse
kinematics capability using the MoveIt! planner backend.


Available Methods
=================

.. currentmodule:: compas_fab.backends

:meth:`MoveItPlanner.inverse_kinematics` calculates
the inverse kinematics for a given target and robot cell state. It returns a single
solution for the inverse kinematics problem. The returned solution will be close to the
starting configuration (as specified in the robot cell state) if the target is reachable
from that configuration. The following example shows how to calculate the inverse
kinematics for a target using the UR5 robot loaded from the running ROS server.

.. literalinclude :: files/04_ik.py
   :language: python

If this function is being called multiple times with exactly the same inputs, an
underlying cache mechanism will be used to return a different solution each time, thus
behaving similar to the ``iter_inverse_kinematics`` method. The example below shows this behavior.

.. literalinclude :: files/04_ik_cache.py
   :language: python

:meth:`MoveItPlanner.iter_inverse_kinematics` returns a
generator that yields multiple solutions for the inverse kinematics problem.
The first solution is sensitive to the starting configuration, while the subsequent
solutions are results of random restarts.

.. literalinclude :: files/04_iter_ik.py
   :language: python

Planner Behavior
================

The default inverse kinematics solver used by the MoveIt! planner is the KDL solver.
(This is defined in the MoveIt! configuration file `kinematics.yaml`).
The KDL solver is a numerical solver that uses approximation method to solve the inverse
kinematics problem. Therefore it is sensitive to the initial guess (the starting
configuration). A random restart strategy is used internally in KDL, and also in the
``iter_inverse_kinematics`` method for finding different solutions. It is not possible
to disable randomness in the KDL solver or to control the random seed used.

Regarding randomness, if the initial guess is close to the solution and the solver can
converge to the same solution, the first solution returned by ``iter_inverse_kinematics``
generator will behave predictably. However, if the initial guess cannot converge to the
solution or that the solution is in collision, the solver will restart and try again.
The resulting first solution will be behave stochastically.

When using the ``iter_inverse_kinematics`` method, if the solver cannot find even one
solution after ``max_attempts`` attempts, it will raise a
:class:` ~compas_fab.backends.InverseKinematicsError` exception.

.. literalinclude :: files/04_ik_unreachable.py
   :language: python

The solver obeys joint limits specified in the URDF
(:attr:`Joint.limit<compas_robots.model.Joint>`), and it will use the safety limits
if they are specified (:attr:`Joint.safety_controller<compas_robots.model.Joint>`).
In addition, if collision detection is enabled, the solver will reject converged
solutions that are in collision with the environment.


Collision checking is turned on by default, which checks for both self-collision and
collision with the environment. If the solver cannot find a solution that is collision
free, it will also raise an :class:`~compas_fab.backends.InverseKinematicsError` exception.
During debugging it is useful to know whether the IK solver is failing due to collision
checking or because the target is not reachable. The user can disable collision checking
by setting the ``allow_collision`` parameter to ``True`` in the ``options`` dictionary.
If the solver still fails after disabling collision checking, the user can be sure that
the problem is not caused by a collision.

.. literalinclude :: files/04_ik_allow_collision.py
   :language: python

By default, the returned  :class:`~compas_robots.Configuration` will only include the
joints that are part of the planning group because other kinematics joints are not
changed when they are not part of the planning group. If the user wants to include all
joints in the returned configuration, the ``return_full_configuration`` parameter can be
set to ``True`` in the ``options`` dictionary. The following example hows how to set the
``return_full_configuration`` parameter to ``True``, however, it will not have an effect
with the UR5 robot since it's only planning group includes all joints.

.. literalinclude :: files/04_ik_full_config.py
   :language: python

The iter_inverse_kinematics method will return multiple solutions using the random
restart mechanism, these solutions are unique to each other following the
``solution_uniqueness_threshold_prismatic`` and
``solution_uniqueness_threshold_revolute`` parameters in the ``options`` dictionary.
Solutions that are not unique will be discarded. The generator will stop yielding
solutions after ``max_results`` attempts. Note that due to the random nature, the
exhaustion of the generator does not imply that all solutions have been found.

.. literalinclude :: files/04_iter_ik_unique.py
   :language: python

Other Examples
==============

The following example uses a robot cell with an attached tool and a workpiece, showing
the possibility to specify PCF, TCF, and OCF as the target by setting
:class:`compas_fab.robots.TargetMode`. See :ref:`Core Concepts: Target Mode<target_mode>`
and :ref:`Core Concepts: Coordinate Frame<coordinate_frames>` for more information.

.. literalinclude :: files/04_ik_target_mode.py
   :language: python


See also
========

- `GetPositionIK Service in MoveIt Noetic<https://docs.ros.org/en/noetic/api/moveit_msgs/html/srv/GetPositionIK.html>`_