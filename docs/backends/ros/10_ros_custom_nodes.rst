
*******************************************************************************
Advanced ROS Concepts
*******************************************************************************


Without a context manager
=========================

If you prefer to work without a context manager, the equivalent code is:

.. code-block:: python

   from compas_fab.backends import RosClient
   client = RosClient()
   client.run()
   print('Connected: ', client.is_connected)
   client.close()

.. note::

    ``RosClient`` also supports using *callbacks* instead of blocking calls.

    A *callback* is a function that is passed to another function as a
    parameter such that the latter function can call the former at any time
    during its own execution. Whenever code needs to respond to an event,
    one of the easiest ways to achieve it is to pass a *callback* function
    that the framework will invoke when the event is fired.

Hello World
===========

The ``Hello world`` of ROS is to start two nodes: a talker and a listener.
The nodes are extremely simple but they exemplify a distributed system with
communication between two processes over the ROS infrastructure.

Writing the talker node
-----------------------

The following example starts a ROS node and begins to publish
messages in loop (to terminate, press ``ctrl+c``):

.. literalinclude :: files/10_ros_hello_world_talker.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Talker node (.PY) <files/10_ros_hello_world_talker.py>`

.. raw:: html

    </div>
    </div>

Writing the listener node
-------------------------

Now let's move on to the listener side:

.. literalinclude :: files/10_ros_hello_world_listener.py
   :language: python

.. raw:: html

    <div class="card bg-light">
    <div class="card-body">
    <div class="card-title">Downloads</div>

* :download:`Listener node (.PY) <files/10_ros_hello_world_listener.py>`

.. raw:: html

    </div>
    </div>

Running the example
-------------------

Open a command prompt and start the talker:

::

    python 01_ros_hello_world_talker.py


Now open a second command prompt and start the listener:

::

    python 01_ros_hello_world_listener.py

You should see the listener printing every time it hears the other node talking.

.. note::

    It is not relevant where the files are located. They can be in different
    folders or even in different computers as long as the ROS master is the same.
