.. _ros_examples:

********************************************************************************
Using ROS
********************************************************************************

The following examples demonstrate the `ROS <http://www.ros.org/>`_
backend.

Before running them, make sure you have configured the
:ref:`ROS backend <ros_backend>` correctly.


First step
==========

The first step is to connect to ROS to verify that the system is working.

Copy and paste the following example into any Python environment
(a standalong script, a CAD environment, etc) and run it, you should
see the output ``Connected: True`` if everything is working properly:

.. code-block:: python

    from compas_fab.backends import RosClient
    client = RosClient()

    def hello_ros():
        print('Connected: %s' % client.is_connected)
        client.terminate()

    client.on_ready(hello_ros)
    client.run_forever()


The first thing to notice in this example is that the ROS client relies
on **callbacks** to work. This allows to use event-based programming and
build more reactive software that does not need to block when it is not
required.

.. note::

    A *callback* is a function that is passed to another function as a
    parameter such that the latter function can call the former at any time
    during its own execution. Whenever code needs to respond to an event,
    one of the easiest ways to achieve it is to pass a *callback* function
    that the framework will invoke when the event is fired.


.. TODO: Add listener/chatter rosclient-based example
.. TODO: Add FK/IK/PathPlan examples
