.. _vrep_examples:

********************************************************************************
Simulation examples with V-REP
********************************************************************************

The following examples demonstrate the `V-REP <http://www.coppeliarobotics.com/>`_
simulation backend. They are based on a sample scene of the RFL
(Robotic Fabrication Lab).

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
        print ('Connected: %s' % client.is_connected())


Forward Kinematics
====================

Moving robots
-------------

The RFL scene used on these examples has 4 robots that can be
referenced by the identifiers: ``A``, ``B``, ``C`` and ``D``.

.. figure:: 05_rfl.png
    :figclass: figure
    :class: figure-img img-fluid

When planning on a multi-robotic setup, it's important to make sure all robots on the setup of them are positioned correctly
and not colliding with each other at start, otherwise path planning will fail.

The state of a robot is specified as an instance of :class:`compas_fab.robots.Configuration`.

Here's a simple example on how to position two of the robots using forward kinematics:

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

Inverse Kinematics
==================

When the configuration required to reach a certain frame with a robot is not know, we use inverse kinematics
to resolve it and find a suitable pose. The following example shows how to calculate this. In this case, it will
take collisions into account, but does not find a path to the goal pose, only that there is at least one
valid configuration to reach the goal pose.

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    goal_pose       = Frame.from_list([-1.0, 0.0, 0.0, 8.110,
                                       0.0, 0.0, -1.0, 7.020,
                                       0.0, -1.0, 0.0, 1.810])

    with VrepClient() as client:
        robot = rfl.Robot('B')

        config = client.set_robot_pose(robot, goal_pose)
        print('Found valid configuration: ', str(config))

Basic path planning example
---------------------------

Calculating a path plan requires several parameters to be configured in order to start
the process. In its minimal expression, a path planning request must define a start
configuration and a goal pose and rely on defaults for the rest. Here is an example
of such a request:

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.robots import *
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    start_config    = Configuration.from_prismatic_and_revolute_values([8.260, -5.320, -3.690],
                                                                       to_radians([-143, 37, -112, 0, -15, -126]))
    goal_pose       = Frame.from_list([-1.0, 0.0, 0.0, 8.110,
                                       0.0, 0.0, -1.0, 7.020,
                                       0.0, -1.0, 0.0, 1.810])

    with VrepClient() as client:
        robot = rfl.Robot('B')

        client.set_robot_config(robot, start_config)
        path = client.find_path_plan(robot, goal_pose)
        print('Found path of %d steps' % len(path))


Complete path planning example
------------------------------

The following example showcases a lot of the configuration options available when
calculating a path plan:

.. code-block:: python

    import logging

    from compas.geometry import Frame
    from compas.datastructures import Mesh

    from compas_fab.robots import *
    from compas_fab.robots import rfl
    from compas_fab.backends import VrepClient

    # Configure logging to DEBUG to see detailed timing of the path planning
    logging.basicConfig(level=logging.DEBUG)

    # Configure parameters for path planning
    start_pose      = Frame.from_list([0.0, 1.0, 0.0, 7.453,
                                       -1.0, 0.0, 0.0, 10.919,
                                       0.0, 0.0, 1.0, 0.609])
    goal_pose       = Frame.from_list([-1.0, 0.0, 0.0, 8.110,
                                       8.97e-13, 0.0, -1.0, 6.920,
                                       0.0, -1.0, 0.0, 1.810])
    algorithm       = 'rrtconnect'
    max_trials      = 1
    resolution      = 0.02
    building_member = Mesh.from_obj('timber_beam.obj')
    structure       = [Mesh.from_obj('timber_structure.obj')]
    metric          = [0.1] * 9
    fast_search     = True

    with VrepClient(debug=True) as client:
        robot = rfl.Robot('B', client=client)
        client.pick_building_member(robot, building_member, start_pose)
        path = client.find_path_plan(robot,
                                     goal_pose,
                                     metric_values=metric,
                                     collision_meshes=structure,
                                     algorithm=algorithm,
                                     trials=max_trials,
                                     resolution=resolution,
                                     shallow_state_search=fast_search)

        print('Found path of %d steps' % len(path))

Grasshopper integration
=======================

.. figure:: 05_grasshopper.png
    :figclass: figure
    :class: figure-img img-fluid

Besides the examples above that can be run standalone or inside CAD software, this package contains
a ready-made integration for Grasshopper that allows configuration of most available parameters.

See :download:`this basic example <grasshopper-basic-example.ghx>` and then
:download:`this complete path planning example <grasshopper-path-planner.ghx>` for Grasshopper.
