"""
.. _compas_fabrication:

********************************************************************************
compas_fabrication
********************************************************************************

.. module:: compas_fabrication

Main package for robotics fabrication for the compAS Framework.

.. rubric:: Submodules

.. toctree::
    :maxdepth: 2

    compas_fabrication.fabrication.robots.rfl
    compas_fabrication.fabrication.grasshopper

"""

import os


HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, 'data'))


def _find_resource(filename):
    filename = filename.strip('/')
    return os.path.abspath(os.path.join(DATA, filename))


def get_data(filename):
    return _find_resource(filename)
