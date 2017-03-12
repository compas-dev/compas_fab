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
