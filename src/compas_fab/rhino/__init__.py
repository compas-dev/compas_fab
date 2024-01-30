"""
********************************************************************************
compas_fab.rhino
********************************************************************************

.. currentmodule:: compas_fab.rhino

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ReachabilityMapArtist

Additionally, this package provides deprecated aliases to the
:class:`~compas_rhino.artists.RobotModelArtist` implemented in COMPAS.

"""

import compas

if compas.RHINO:
    from .scene import (
        ReachabilityMapObject,
    )

    __all__ = [
        "ReachabilityMapObject",
    ]
