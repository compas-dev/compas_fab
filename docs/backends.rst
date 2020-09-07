.. _backends:

********************************************************************************
Working with backends
********************************************************************************

.. highlight:: bash

One of the driving principles of **COMPAS** framework is to create an ecosystem
and serve as an interface between different front-ends (e.g. CAD software) and
back-ends.

In the case of **COMPAS FAB**, it provides access to multiple robotic platforms
as backends. This chapter highlights the available ones and explains how to
obtain them and set them up.


Installing backends
===================

Backends can be installed in different ways. Some backends are very simple to
install, while others are very complex.

In order to simplify working with these tools and have a consistent way
to use and test different backends, **COMPAS FAB** provides them as
`Docker containers`_. Docker containers are a way to bundle systems into
isolated, standarized software units with full reproducibility. It greatly
reduces the time it takes to get a backend up and running.

However, all of them can be installed *without* Docker as well. Please refer
to the documentation of the respective tool for standard installation
instructions.

Installing Docker
-----------------

Download and install `Docker Community Edition`_:

* `Docker for Windows`_
* `Docker for Mac`_

.. note::

    Make sure you have enabled virtualization in your BIOS.
    Docker will complain if not.

.. note::

    If you're a Windows user, you will need at least Windows 10 Pro.

    After installation, make sure Docker runs in Linux containers mode: right-click
    the docker icon on the tray bar; if there is an option to ``Switch to Linux containers``,
    **select it** and wait for Docker to switch before moving forward.

Working with containers
-----------------------

For Visual Studio Code users, we recommend installing the ``Docker`` extension.

Alternatively, install `Kitematic` to have a visual overview of the
running containers:

* `Kitematic for Windows`_
* `Kitematic for Mac`_

Developing new backends
=======================

If you are interested in developing/integrating backends to the framework, check
the :ref:`backend architecture document <architecture>` and the
:ref:`contributors_guide`.

Next steps
==========

Check documentation for your backend of choice:

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :glob:

    backends/*


.. _Docker: https://www.docker.com/
.. _Docker Community Edition: https://www.docker.com/get-started
.. _Docker containers: https://www.docker.com/resources/what-container
.. _Docker for Windows: https://hub.docker.com/editions/community/docker-ce-desktop-windows
.. _Docker for Mac: https://hub.docker.com/editions/community/docker-ce-desktop-mac
.. _Kitematic for Windows: https://download.docker.com/kitematic/Kitematic-Windows.zip
.. _Kitematic for Mac: https://download.docker.com/kitematic/Kitematic-Mac.zip
