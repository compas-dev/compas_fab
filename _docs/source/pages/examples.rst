.. _examples:

********************************************************************************
Examples
********************************************************************************

There are multiple ways to run these examples. Most can be executed from any
environment, for instance:

  * From an interactive ``python`` session
  * From an IDE or text editor (e.g. Eclipse, PyCharm, Sublime Text, etc)
  * From inside CAD software (e.g. Rhino, Blender).


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

