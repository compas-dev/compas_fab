"""
.. _compas_fab:

********************************************************************************
compas_fab
********************************************************************************

.. module:: compas_fab

Main package for robotics fabrication for the compas framework.

.. rubric:: Submodules

.. toctree::
    :maxdepth: 2

    compas_fab.fab.geometry
    compas_fab.fab.grasshopper
    compas_fab.fab.robots
    compas_fab.fab.robots.rfl
    compas_fab.fab.sensors

"""

import os


HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, 'data'))


def _find_resource(filename):
    filename = filename.strip('/')
    return os.path.abspath(os.path.join(DATA, filename))


def get_data(filename):
    return _find_resource(filename)
