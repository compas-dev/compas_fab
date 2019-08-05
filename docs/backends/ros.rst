.. _ros_backend:

****************
ROS
****************

.. highlight:: bash

The `Robot Operating System <http://www.ros.org>`_ (ROS) is a flexible framework
for writing robot software. It is a collection of tools, libraries, and
conventions that aim to simplify the task of creating complex and robust
robot behavior across a wide variety of robotic platforms.

Running a ROS system usually involves multiple nodes (i.e. computers, real or
virtual), interconnected through a master controller.

There are at least 3 different ways to run ROS: using Linux, using WSL on
Windows, and using Docker.


ROS on Linux
============

The usual but most involved way to install ROS is on a Linux machine,
either virtual or real. The machine should have an IP address reachable
from your computer.

Follow the `ROS installation instructions`_ for all the details, or
alternatively, use the following commands as a brief outline of the steps
required to install ROS on **Ubuntu 16.04**:

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full ros-kinetic-rosbridge-server ros-kinetic-tf2-web-republisher python-rosinstall python-rosinstall-generator python-wstool

    sudo rosdep init && rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Once ROS is installed, you can start a minimally functional ROS system,
containing a ROS master and the `ROS Bridge`_ with the following command::

    roslaunch rosbridge_server rosbridge_websocket.launch


ROS on WSL
==========

For Windows 10 users, an alternative is to install the
`Windows Subsystem for Linux`_ (WSL). WSL allows to run Linux within
Windows without the need for an additional virtual machine.

To install WSL, open PowerShell as administrator and run:

::

    Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux

Open the Microsoft Store and install ``Ubuntu 16.04`` Linux distribution.
Once the installation is completed, run ``bash`` and follow the instructions
above to install ROS on Linux.

.. seealso::

    For additional details, see `Microsoft WSL documentation`_.


ROS on Docker
=============

To massively simplify the use of these tools, we package complete ROS systems
into bundles of `Docker`_ containers. Each of these bundles runs in a
virtualized network within your computer.

Besides easing deployment, containers have the added benefit of ensuring
repeatability.

Once you made sure `Docker`_ is running, you can run ROS nodes as containers.
Gramazio Kohler Research publishes ROS images on `Docker Hub`_ but there are
many more to be found online.

You can start a minimally functional ROS system, containing a ROS master and
the `ROS Bridge`_ with the following command::

    docker run -p 9090:9090 -t gramaziokohler/ros-base roslaunch rosbridge_server rosbridge_websocket.launch

Complete ROS systems
--------------------

It is usually not enough to run single ROS nodes. ROS systems are networks of
multiple interconnected nodes. Docker provides a way to compose virtualized
networks using the ``docker-compose`` command. These commands take one simple
configuration file as input, and handle all tasks required to run and connect
all the nodes.

As an example, download :download:`this file <files/base/docker-compose.yml>`,
open the command prompt, go to the folder where the file was downloaded,
and run the following command::

    docker-compose up -d

You now have a ROS system with two nodes running: a ROS master and
the `ROS Bridge`_ which adds a web socket channel to communicate with ROS.

Creating new ROS bundles using containers is usually only a matter of combining
them into a new ``docker-compose.yml`` file.

.. _ros_bundles_list:

**List of complete ROS systems**

.. list-table:: Table of ROS systems provided via ``docker compose``
   :widths: 20 8 8 14 18 8 10 14
   :header-rows: 1

   * - Name
     - Core
     - Bridge
     - Planner
     - Robot drivers
     - RViz
     - Web UI [#f1]_
     - Download
   * - Base setup
     - ✅
     - ✅
     - ❌
     - ❌
     - ❌
     - ❌
     - :download:`Link <files/base/docker-compose.yml>`
   * - Panda Demo
     - ✅
     - ✅
     - MoveIt!
     - ❌
     - ✅
     - ✅
     - :download:`Link <files/panda-demo/docker-compose.yml>`
   * - ABB IRB1600 Demo
     - ✅
     - ✅
     - MoveIt!
     - ``abb_driver``
     - ✅
     - ✅
     - :download:`Link <files/abb-irb1600-demo/docker-compose.yml>`
   * - UR3 Planner
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ❌
     - ❌
     - :download:`Link <files/ur3-planner/docker-compose.yml>`
   * - UR5 Planner
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ❌
     - ❌
     - :download:`Link <files/ur5-planner/docker-compose.yml>`
   * - UR10 Planner
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ❌
     - ❌
     - :download:`Link <files/ur10-planner/docker-compose.yml>`
   * - UR3 Demo
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ✅
     - ✅
     - :download:`Link <files/ur3-demo/docker-compose.yml>`
   * - UR5 Demo
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ✅
     - ✅
     - :download:`Link <files/ur5-demo/docker-compose.yml>`
   * - UR10 Demo
     - ✅
     - ✅
     - MoveIt!
     - ``ur_modern_driver``
     - ✅
     - ✅
     - :download:`Link <files/ur10-demo/docker-compose.yml>`

For access to the web UI, start your browser and go to:

::

    http://localhost:8080/vnc.html?resize=scale&autoconnect=true


.. _ROS installation instructions: http://wiki.ros.org/ROS/Installation
.. _Windows Subsystem for Linux: https://docs.microsoft.com/en-us/windows/wsl/about
.. _Microsoft WSL documentation: https://docs.microsoft.com/en-us/windows/wsl/install-win10
.. _Docker: https://www.docker.com/
.. _Docker Hub: https://hub.docker.com/r/gramaziokohler/
.. _ROS Bridge: http://wiki.ros.org/rosbridge_suite

Next Steps
==========

* :ref:`Examples: Description models <examples_description_models>`
* :ref:`Examples: ROS Backend <examples_ros>`
* :ref:`COMPAS FAB API Reference <reference>`

.. rubric:: Footnotes

.. [#f1] Web UI is based on `NoVNC <https://novnc.com/>`_.
