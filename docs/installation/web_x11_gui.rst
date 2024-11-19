.. _docker_gui:

********************************************************************************
Access Docker Backend GUI
********************************************************************************

.. highlight:: bash

When using Docker or WSL to run backends, there is no visible graphical
user interface (GUI). **COMPAS FAB** aims at making this unnecessary, but
if desired, there are two possibilities to open the graphical user interface
of a backend.

Visualization over web browser
==============================

This is the easiest option, and it is normally only available for backends
based on Docker. The display of the backend is forwarded to a container that
serves it over a web connection, making it available from your browser.

Not all Docker setups will include this ability, but if they do, it can be
accessed easily. Start your browser and go to the following address:

::

    http://localhost:8080/vnc.html?resize=scale&autoconnect=true


This feature is possible thanks to `NoVNC <https://novnc.com/>`_.


Visualization forwarding display
================================

This option allows to forward a display directly to your operating system.
However, it is important to note this is not entirely supported in all platforms.

First install an ``X11`` server:

* `XMing For Windows <https://sourceforge.net/projects/xming/>`_
* `XQuartz For Mac <https://www.xquartz.org/>`_ (see here for `more details <https://medium.com/@mreichelt/how-to-show-x11-windows-within-docker-on-mac-50759f4b65cb>`_).

Configure ``X11`` security:

* On Windows, add your IP address to the file
  ``%ProgramFiles(x86)%\XMing\X0.hosts``. It needs to be opened
  as administrator.
* On Mac, run ``xhost +local:root``. Remember to disable later with
  ``xhost -local:root``.

Finally, set the ``DISPLAY`` variable to point to your ``X11`` server as follows
replacing ``YOUR_IP_ADDRESS`` with your current IP.

For usage from WSL, set it using the following command:

::

    export DISPLAY=YOUR_IP_ADDRESS:0.0

For usage from Docker, set it adding the following environment
variable to the ``moveit`` service in the ``docker-compose.yml`` file:

::

    - DISPLAY=host.docker.internal:0.0
