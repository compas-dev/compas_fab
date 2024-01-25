.. _ros_backend:

****************
ROS
****************

.. highlight:: bash

The `Robot Operating System <https://www.ros.org>`_ (ROS) is a flexible framework
for writing robot software. It is a collection of tools, libraries, and
conventions that aim to simplify the task of creating complex and robust
robot behavior across a wide variety of robotic platforms.

Running a ROS system usually involves multiple nodes (i.e. computers, real or
virtual), interconnected through a master controller.

There are at least 3 different ways to run ROS: using Docker, using Linux, and
using WSL on Windows. In recent times, it became possible to install ROS using
Conda on Windows as well.


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

    docker run -p 9090:9090 -t gramaziokohler/ros-noetic-base roslaunch rosbridge_server rosbridge_websocket.launch

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
them into a new ``docker-compose.yml`` file, which is relatively simple but we
prepared some very common ones as examples.

.. _ros_bundles_list:

**Complete ROS system examples**

* ROS Noetic Base setup: :download:`Link <files/base/docker-compose.yml>`
* ABB IRB120: :download:`Link <files/abb-irb120-demo/docker-compose.yml>`
* ABB IRB120T: :download:`Link <files/abb-irb120t-demo/docker-compose.yml>`
* ABB IRB1600: :download:`Link <files/abb-irb1600-demo/docker-compose.yml>`
* ABB IRB4600 40/255: :download:`Link <files/abb-irb4600_40_255-demo/docker-compose.yml>`
* ABB IRB4600 60/205: :download:`Link <files/abb-irb4600_60_205-demo/docker-compose.yml>`
* Panda: :download:`Link <files/panda-demo/docker-compose.yml>`
* RFL: :download:`Link <files/rfl-demo/docker-compose.yml>`
* UR3: :download:`Link <files/ur3-demo/docker-compose.yml>`
* UR3e: :download:`Link <files/ur3e-demo/docker-compose.yml>`
* UR5: :download:`Link <files/ur5-demo/docker-compose.yml>`
* UR5e: :download:`Link <files/ur5e-demo/docker-compose.yml>`
* UR10: :download:`Link <files/ur10-demo/docker-compose.yml>`
* UR10e: :download:`Link <files/ur10e-demo/docker-compose.yml>`

Once the containers are running, it is possible to access the graphic user interface.
Check :ref:`the following page <backends_gui>` for more details.

ROS on Linux
============

The usual but most involved way to install ROS is on a Linux machine,
either virtual or real. The machine should have an IP address reachable
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
Check :ref:`the following page <backends_gui>` for more details.

.. seealso::

    For additional details, see `Microsoft WSL documentation`_.



.. _ROS installation instructions: https://wiki.ros.org/ROS/Installation
.. _Windows Subsystem for Linux: https://docs.microsoft.com/en-us/windows/wsl/about
.. _Microsoft WSL documentation: https://docs.microsoft.com/en-us/windows/wsl/install-win10
.. _Docker: https://www.docker.com/
.. _Docker Hub: https://hub.docker.com/u/gramaziokohler/
.. _ROS Bridge: https://wiki.ros.org/rosbridge_suite

Next Steps
==========

* `Tutorial: COMPAS Robots <https://compas.dev/compas/1.17.9/tutorial/robots.html>`__
..
  TODO: whatever user intersphinx link for compas compas robots tutorial
  Something like this: * :ref:`Tutorial: COMPAS Robots <compas:robots>`
* :ref:`Examples: Description models <examples_description_models>`
* :ref:`Examples: ROS Backend <examples_ros>`
* :ref:`COMPAS FAB API Reference <reference>`
