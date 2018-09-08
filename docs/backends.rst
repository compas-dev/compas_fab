.. _backends:

********************************************************************************
Working with backends
********************************************************************************

One of the driving principles of **COMPAS** framework is to create an ecosystem
and serve as an interface between different front-ends (e.g. CAD software) and
back-ends.

In the case of **compas_fab**, it provides access to multiple robot platforms
as backends. This chapter highlights the available ones and explains how to
obtain them and set them up.

In order to make it easier to work with these tools and have a consistent way
to use and test different backends, the framework provides packaged versions of them.
These entire systems are packaged into `Docker containers`_. Docker containers are
a way to package systems into isolated, standarized software units with full
reproducibility.


Pre-requisite
=============

As a first step, install `Docker`_ on your system. If you're a Windows user,
you will need at least Windows 10. Using Docker greatly simplifies the setup
of these tools, otherwise each of them is potentially a very time consuming task.

Download `Docker Community Edition`_:

* `Docker for Windows <https://store.docker.com/editions/community/docker-ce-desktop-windows>`_
* `Docker for Mac <https://store.docker.com/editions/community/docker-ce-desktop-mac>`_

Optionally, install `Kitematic` to have a visual overview of the running containers:

* `Kitematic for Windows <https://download.docker.com/kitematic/Kitematic-Windows.zip>`_
* `Kitematic for Mac <https://download.docker.com/kitematic/Kitematic-Mac.zip>`_

.. note::

    Make sure you have enabled virtualization in your BIOS. Docker will complain if not.


Backends
========

Once Docker is installed on your system, you can check out the different
backends made available:

.. toctree::
    :maxdepth: 1
    :titlesonly:
    :glob:

    backends/*


Links
=====

* `Docker Community Edition`_
* `Docker: What is a container <https://www.docker.com/resources/what-container>`_
* `V-REP container, generic image <https://hub.docker.com/r/gramaziokohler/vrep/>`_

.. _Docker: https://www.docker.com/
.. _Docker Community Edition: https://www.docker.com/get-started
.. _Docker containers: https://www.docker.com/resources/what-container
