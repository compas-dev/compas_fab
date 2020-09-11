"""
********************************************************************************
compas_fab.artists
********************************************************************************

.. currentmodule:: compas_fab.artists

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BaseRobotModelArtist


"""

from __future__ import absolute_import

from compas_fab.artists.base import BaseRobotModelArtist

__all__ = [
    'BaseRobotModelArtist'
]
