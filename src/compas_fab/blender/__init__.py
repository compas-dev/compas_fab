"""
********************************************************************************
compas_fab.blender
********************************************************************************

.. currentmodule:: compas_fab.blender

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

This package provides a robot artists implementation that is optimized to use
in Blender.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RobotModelArtist

"""

from .artists import RobotArtist
from .artists import RobotModelArtist

__all__ = ['RobotArtist', 'RobotModelArtist']
