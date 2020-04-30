********************************************************************************
Simulation examples with V-REP
********************************************************************************

The following examples demonstrate the `V-REP <http://www.coppeliarobotics.com/>`_
simulation backend. They are based on a sample scene of the Robotic Fabrication
Lab (RFL).

Before running them, make sure you have configured the
:ref:`V-REP backend <vrep_backend>` correctly.

First step
==========

The first step is just connect to the simulator and verify the connection
is working.

Copy and paste the following example into any Python environment
(a standalong script, a CAD environment, etc) and run it, you should
see the output ``Connected: True`` if everything is working properly:

.. code-block:: python

    from compas_fab.backends import VrepClient

    with VrepClient() as client:
        print ('Connected: %s' % client.is_connected)


Forward Kinematics
====================

Moving robots
-------------

The RFL scene used on these examples has 4 robots that can be
referenced by the identifiers: ``A``, ``B``, ``C`` and ``D``.

.. figure:: files/01_rfl.png
    :figclass: figure
    :class: figure-img img-fluid

When planning on a multi-robotic setup, it's important to make sure all robots
on the setup of them are positioned correctly and not colliding with each other
at start, otherwise path planning will fail.

The state of a robot is specified as an instance of
:class:`compas_fab.robots.Configuration`.

Here's a simple example on how to position two of the robots using
forward kinematics:

.. code-block:: python

    from compas_fab.robots import *
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    config_robot_a    = Configuration.from_prismatic_and_revolute_values([8.260, -1.000, -3.690],
                                                                         to_radians([190, 0, 0, 0, 90, 0]))

    config_robot_b    = Configuration.from_prismatic_and_revolute_values([8.260, -8.320, -3.690],
                                                                         to_radians([190, 0, 0, 0, 90, 0]))

    with VrepClient() as client:
        robot_a = rfl.Robot('A')
        robot_b = rfl.Robot('B')
        client.set_robot_config(robot_a, config_robot_a)
        client.set_robot_config(robot_b, config_robot_b)

        frame_a = client.forward_kinematics(robot_a)
        frame_b = client.forward_kinematics(robot_b)
        print('End effector poses: ', str(frame_a), str(frame_b))

Inverse Kinematics
==================

When the configuration required to reach a certain frame with a robot is not
know, we use inverse kinematics to resolve it and find a suitable pose.
The following example shows how to calculate this. In this case, it will
take collisions into account, but does not find a path to the goal pose, only
that there is at least one valid configuration to reach the goal pose.

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    goal_pose = Frame((8.110, 7.020, 1.810), (-1, 0, 0), (-0, -0, -1))

    with VrepClient() as client:
        robot = rfl.Robot('B')
        configs = client.inverse_kinematics(robot, goal_pose)

        assert len(configs) > 0, 'No IK solution found'
        print('Found valid configuration: ', str(configs[-1]))

Basic path planning example
---------------------------

Calculating a path plan requires several parameters to be configured in order
to start the process. In its minimal expression, a path planning request must
define a start configuration and a goal pose and rely on defaults for the rest.

Here is an example of such a request:

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.robots import *
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    start_config    = Configuration.from_prismatic_and_revolute_values([8.260, -5.320, -3.690],
                                                                       to_radians([-143, 37, -112, 0, -15, -126]))
    goal_pose       = Frame((8.110, 7.020, 1.810), (-1, 0, 0), (-0, -0, -1))

    with VrepClient() as client:
        robot = rfl.Robot('B')
        client.set_robot_config(robot, start_config)
        path = client.plan_motion(robot, goal_pose)
        print('Found path of %d steps' % len(path))


Or Copy & Paste the following code into a Python file:

.. literalinclude :: files/01_complete_path_planning_example.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Path planning example (.PY) <files/01_complete_path_planning_example.py>`

.. raw:: html

    </div>
    </div>


Grasshopper integration
=======================

.. figure:: files/01_grasshopper.png
    :figclass: figure
    :class: figure-img img-fluid

Besides the examples above that can be run standalone or inside CAD software,
this package contains a ready-made integration for Grasshopper that allows
configuration of most available parameters.

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Basic path planning example <files/01_grasshopper_basic_example.ghx>`
* :download:`Complete path planning example <files/01_grasshopper_path_planner.ghx>`.

.. raw:: html

    </div>
    </div>
