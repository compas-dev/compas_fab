.. _vrep_backend:

******************
V-REP
******************

`V-REP <http://www.coppeliarobotics.com/>`_ is a robotic simulation tool
by Coppelia Robotics with an integrated development environment and various
programmable interfaces.

In order to use it as a backend for **compas_fab**, it is possible to download
and install it as a normal application (see details below), but it can also be executed as a service
inside a container in only a few steps with the added benefit of ensuring
repeatability.

Once you made sure `Docker`_ is running, you can download and install the V-REP
container with the following commands on the command line::

    docker pull gramaziokohler/vrep-rfl
    docker run --restart=always -p 19997:19997 -d gramaziokohler/vrep-rfl

.. note::

    This container includes a preloaded sample 3D scene. To use a different
    scene, download the generic container: ``gramaziokohler/vrep`` from `Docker Hub`_.

.. note::

    If your operating system does not support running Docker, an alternative is
    is to `download V-REP <http://www.coppeliarobotics.com/downloads.html>`_ and
    install it as a normal application.

.. _Docker: https://www.docker.com/
.. _Docker Hub: https://hub.docker.com/r/gramaziokohler/vrep/

Next Steps
==========

* :ref:`Examples <examples>`
* :ref:`API Reference <reference>`
