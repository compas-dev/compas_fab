.. _setup_backend_ros:

*******************************************************************************
Setup ROS MoveIt!
*******************************************************************************

.. highlight:: bash

The `Robot Operating System <https://www.ros.org>`_ (ROS) is a flexible framework
for writing robot software. It is a collection of tools, libraries, and
conventions that aim to simplify the task of creating complex and robust
robot behavior across a wide variety of robotic platforms.
A ROS system typically involves multiple nodes (i.e. different software services
running on real or virtual computers), interconnected through a master controller.

The `MoveIt! package <https://moveit.ai/>`_ is a ROS package that provides
services for robotic motion planning, manipulation, and perception.
It is a widely used software for manipulation and mobile robots.

**COMPAS FAB** provides :class:`compas_fab.backends.RosClient` for communicating
with a ROS system via RosBridge over WebSockets. This client is used to send and
receive data from the ROS modes. The :class:`compas_fab.backends.MoveItPlanner`
relies the RosClient to interact with the MoveIt planning service in ROS, which
exposes these services to the **COMPAS FAB** user.

Before using ROS and MoveIt, the ROS system must be installed.
There are at least 3 different ways to run ROS: using Docker, using Linux, and
using WSL on Windows. In recent times, it became possible to install ROS using
Conda on Windows as well.


Installing ROS with Docker
==========================

In order to simplify working with ROS in Ubuntu and have a consistent way
to use and test different backends, **COMPAS FAB** provides ROS and MoveIt as
`Docker containers`_. Docker containers are a way to bundle systems into
isolated, standardized software units with full reproducibility. It greatly
reduces the time it takes to get a backend up and running.


Installing Docker
-----------------

Download and install `Docker Desktop`_:

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


For Visual Studio Code users, we recommend installing the ``Docker`` extension.
It will allow you to right-click on a .yml file and select ``Compose Up`` to
start the containers.

ROS on Docker
-------------

We package complete ROS systems
into bundles of `Docker`_ containers. Each of these bundles runs in a
virtualized network within your computer.

Besides easing deployment, containers have the added benefit of ensuring
repeatability.

Once you made sure `Docker`_ is running, you can run ROS nodes as containers.
Gramazio Kohler Research publishes ROS images on `Docker Hub`_ but there are
many more to be found online.

You can start a minimally functional ROS system, containing a ROS master and
the `ROS Bridge`_ with the following command::

    docker run -p 9090:9090 -t gramaziokohler/ros-noetic-base roslaunch rosbridge_server rosbridge_websocket.launch

Running ROS and MoveIt with a specific Robot
--------------------------------------------

It is usually not enough to run single ROS nodes. ROS systems are networks of
multiple interconnected nodes. Docker provides a way to compose virtualized
networks using the ``docker-compose`` command. These commands take one simple
configuration file as input, and handle all tasks required to run and connect
all the nodes.

As an example, download :download:`this file <docker_files/base/docker-compose.yml>`,
open the command prompt, go to the folder where the file was downloaded,
and run the following command::

    docker-compose up -d

You now have a ROS system with two nodes running: a ROS master and
the `ROS Bridge`_ which adds a web socket channel to communicate with ROS.

Creating new ROS bundles using containers is usually only a matter of combining
them into a new ``docker-compose.yml`` file, which is relatively simple but we
prepared some very common ones as examples.

.. _ros_bundles_list:

**ROS system with example robots**

* ROS Noetic Base setup: :download:`Link <docker_files/base/docker-compose.yml>`
* ABB IRB120: :download:`Link <docker_files/abb-irb120-demo/docker-compose.yml>`
* ABB IRB120T: :download:`Link <docker_files/abb-irb120t-demo/docker-compose.yml>`
* ABB IRB1600: :download:`Link <docker_files/abb-irb1600-demo/docker-compose.yml>`
* ABB IRB4600 40/255: :download:`Link <docker_files/abb-irb4600_40_255-demo/docker-compose.yml>`
* ABB IRB4600 60/205: :download:`Link <docker_files/abb-irb4600_60_205-demo/docker-compose.yml>`
* Panda: :download:`Link <docker_files/panda-demo/docker-compose.yml>`
* RFL: :download:`Link <docker_files/rfl-demo/docker-compose.yml>`
* UR3: :download:`Link <docker_files/ur3-demo/docker-compose.yml>`
* UR3e: :download:`Link <docker_files/ur3e-demo/docker-compose.yml>`
* UR5: :download:`Link <docker_files/ur5-demo/docker-compose.yml>`
* UR5e: :download:`Link <docker_files/ur5e-demo/docker-compose.yml>`
* UR10: :download:`Link <docker_files/ur10-demo/docker-compose.yml>`
* UR10e: :download:`Link <docker_files/ur10e-demo/docker-compose.yml>`

Once the containers are running, it is possible to access the graphic user interface.
See :ref:`this page <docker_gui>` for more details.

ROS on Linux
============

The usual but most involved way to install ROS is on a Linux machine,
either virtual or real. The machine must have an IP address reachable
from your computer.

Follow the `ROS installation instructions`_ for all the details, or
alternatively, use the following commands as a brief outline of the steps
required to install ROS on **Ubuntu 20.04**:

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full ros-noetic-rosbridge-server python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

    sudo rosdep init && rosdep update
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Once ROS is installed, you can start a minimally functional ROS system,
containing a ROS master and the `ROS Bridge`_ with the following command::

    roslaunch rosbridge_server rosbridge_websocket.launch

You will need to setup your own robot models and MoveIt configurations to
use the MoveItPlanner with your robot.

ROS on WSL
==========

For Windows 10 users, an alternative is to install the
`Windows Subsystem for Linux`_ (WSL). WSL allows to run Linux within
Windows without the need for an additional virtual machine.

To install WSL, open PowerShell as administrator and run:

::

    wsl --install

This command will enable the required optional components, download the latest Linux kernel,
set WSL 2 as your default, and install a Linux distribution for you.
Once the installation is completed, run ``bash`` and follow the instructions
above to install ROS on Linux.

After installation, it is possible to access the graphic user interface.
Check :ref:`this page <docker_gui>` for more details.

.. seealso::

    For additional details, see `Microsoft WSL documentation`_.



.. _ROS installation instructions: https://wiki.ros.org/ROS/Installation
.. _Windows Subsystem for Linux: https://docs.microsoft.com/en-us/windows/wsl/about
.. _Microsoft WSL documentation: https://docs.microsoft.com/en-us/windows/wsl/install-win10
.. _Docker: https://www.docker.com/
.. _Docker Hub: https://hub.docker.com/u/gramaziokohler/
.. _ROS Bridge: https://wiki.ros.org/rosbridge_suite

.. _Docker Desktop: https://www.docker.com/get-started
.. _Docker containers: https://www.docker.com/resources/what-container
.. _Docker for Windows: https://hub.docker.com/editions/community/docker-ce-desktop-windows
.. _Docker for Mac: https://hub.docker.com/editions/community/docker-ce-desktop-mac

Next Steps
==========

* :doc:`Tutorial: COMPAS Robots <compas_robots:tutorial>`
* :ref:`Examples: Description models <examples_description_models>`
* :ref:`Examples: ROS Backend <examples_ros>`
* :ref:`COMPAS FAB API Reference <reference>`
