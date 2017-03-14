.. _examples_rfl:

********************************************************************************
RFL Simulation examples
********************************************************************************

.. contents::

RFL simulation requires a running instance of `v-rep <http://www.coppeliarobotics.com/>`_
with the RFL scene pre-loaded. There are two options to run v-rep:

 * `Download it <http://www.coppeliarobotics.com/downloads.html>`_, install it
   and open the RFL scene file contained in this package
   (``fabrication\robots\rfl\vrep_remote_api\rfl_scene.ttt``).
 * Install as a service using Docker (only on Windows 10 and higher):

   * Make sure you have `Docker <https://www.docker.com/>`_ installed.
   * Run the following commands on the command line::

        docker pull gramaziokohler/vrep-rfl:3.3
        docker run -p 19997:19997 -d gramaziokohler/vrep-rfl:3.3

Basic example
=============

The most basic example is just connect to the simulator and verify the connection
is working.

Copy and paste the following example and run it, you should see ``Connected: True``
if everything is working properly::

    from compas_fabrication.fabrication.robots.rfl import *

    with Simulator() as simulator:
        print ('Connected: %s' % simulator.is_connected())


Moving robots
=============

The RFL has 4 robots that can be referenced by letter: ``A``, ``B``, ``C`` and ``D``.

It's important to make sure all four of them are positioned correctly and not colliding with each other at start, otherwise path planning will fail.

The position of a robot is specified as an instance of :class:`compas_fabrication.fabrication.robots.rfl.Configuration`.

Here's a simple example on how to position two of the robots::

    from compas_fabrication.fabrication.robots.rfl import *

    config_robot_a    = Configuration(coordinates=[8.26, -1, -3.69],
                                    joint_values=[190, 0, 0, 0, 90, 0])
    config_robot_b    = Configuration(coordinates=[8.26, -8.32, -3.69],
                                    joint_values=[190, 0, 0, 0, 90, 0])

    with Simulator() as simulator:
        robot_a = Robot('A', client=simulator)
        robot_b = Robot('B', client=simulator)

        simulator.set_robot_config(robot_a, config_robot_a)
        simulator.set_robot_config(robot_b, config_robot_b)


Minimal path planning example
=============================

Calculating a path plan requires several parameters to be configured in order to start
the process. In its minimal expression, a path planning request must define a start
configuration and a goal pose and rely on defaults for the rest. Here is an example
of such a request::

    from compas_fabrication.fabrication.robots.rfl import *

    start_config    = Configuration(coordinates=[8.26, -5.32, -3.69],
                                    joint_values=[-143, 37, -112, 0, -15, -126])
    goal_pose       = [-1.0, 0.0, -8.97e-13, 8.11,
                       8.97e-13, 0.0, -1.0, -7.02,
                       0.0, -1.0, 0.0, -1.81]

    with Simulator() as simulator:
        robot = Robot('B', client=simulator)

        simulator.set_robot_config(robot, start_config)
        path = simulator.find_path_plan(robot, goal_pose)
