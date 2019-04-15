.. _ros_backend:

****************
ROS
****************

The Robot Operating System (`ROS <http://www.ros.org/>`_) is a flexible framework
for writing robot software. It is a collection of tools, libraries, and conventions
that aim to simplify the task of creating complex and robust robot behavior across
a wide variety of robotic platforms.

Running a ROS system usually involves multiple nodes (i.e. computers, real or virtual),
interconnected through a master controller. To massively simplify the use of these
tools, we package entire ROS systems into sets of `Docker`_ containers. Each of
these sets runs in a virtualized network within your computer.

In order to run a ROS system that includes a graphical interface, first make sure
to install and start an ``X11`` server:

* `XMing For Windows <https://sourceforge.net/projects/xming/>`_
* `XQuartz For Mac <https://www.xquartz.org/>`_ (see here for `more details <https://medium.com/@mreichelt/how-to-show-x11-windows-within-docker-on-mac-50759f4b65cb>`_).

Besides simplifying the deployment, using containers has the added benefit of ensuring
repeatability.

Once you made sure `Docker`_ is running, you can pull and run some ROS
containers. The **COMPAS FAB** team publishes some images on `Docker Hub`_
but there are many more to be found online. You can start a single ROS Master
with the following commands on the command prompt::

    docker pull gramaziokohler/ros-base
    docker run -p 11311:11311 -t gramaziokohler/ros-base roscore

Entire ROS systems
==================

It is usually not enough to run single ROS nodes. ROS systems are networks of
multiple interconnected nodes. Docker provides a way to compose virtualized networks
using the ``docker-compose`` command. These commands takes one simple configuration
file as input, and take care of pulling, running and connecting all the nodes.

Download :download:`this basic ROS system <ros-basic/docker-compose.yml>` into any
folder of your computer, open the command prompt, go to the folder where the file was
downloaded, and run the following command::

    docker-compose up -d

You now have a ROS system with two nodes running: ``roscore`` (**master**) and
the `ROS Bridge`_ which adds a web socket channel to communicate to ROS.

.. note::

    An alternative way to install ROS on Windows is to use
    `WSL: Windows Subsystem for Linux <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_.

.. _Docker: https://www.docker.com/
.. _Docker Hub: https://hub.docker.com/r/gramaziokohler/
.. _ROS Bridge: http://wiki.ros.org/rosbridge_suite

Next Steps
==========

* :ref:`Examples <examples>`
* :ref:`API Reference <reference>`
