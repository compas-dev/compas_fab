.. _gettingstarted:

********************************************************************************
Getting Started
********************************************************************************

This package builds on top of the `compas framework <http://block.arch.ethz.ch/docs/compas/core/>`_.
Make sure you followed `the installation steps described here <http://block.arch.ethz.ch/docs/compas/core/pages/gettingstarted.html>`_
to get the framework running and have the folder structure in place to install additional packages.

Once the setup is ready, clone the `compas_fab <https://bitbucket.org/GramazioKohlerResearch/compas_fab>`_
package inside the ``packages`` folder. The end result should look like::

    .../compas/core
    .../compas/packages
    .../compas/packages/compas_fab


Install dependencies
====================

Dependencies are installed using ``pip``. Open a terminal window, change to the
folder where you cloned this repository and run the following::

    pip install -r requirements.txt

 
You're ready to go!

Check out the :ref:`Examples <examples>` to get started with code and 
the :ref:`API documentation <api>` for an overview of what's inside the package.
