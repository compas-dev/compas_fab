"""
********************************************************************************
compas_fab
********************************************************************************

.. currentmodule:: compas_fab

This library provides tools to plan and execute robotic fabrication processes using
the COMPAS Framework with a special focus on enabling its usage from within CAD
environments.

.. toctree::
    :maxdepth: 3

    compas_fab.geometry
    compas_fab.ghpython
    compas_fab.robots
    compas_fab.sensors
    compas_fab.utilities

"""

import os

from .__version__ import __author__, __author_email__, __copyright__, __description__, __license__, __title__, __url__, __version__

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, 'data'))


def _find_resource(filename):
    filename = filename.strip('/')
    return os.path.abspath(os.path.join(DATA, filename))


def get_data(filename):
    return _find_resource(filename)


__all__ = ['__author__', '__author_email__', '__copyright__', '__description__', '__license__', '__title__', '__url__', '__version__', 'get_data']
