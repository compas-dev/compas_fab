"""
********************************************************************************
compas_fab.ghpython
********************************************************************************

.. currentmodule:: compas_fab.ghpython

Scene objects
-------------

In **COMPAS**, the `scene objects` are classes that assist with the visualization
of datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ReachabilityMapObject


"""

import compas

if compas.RHINO:
    from .scene import (
        ReachabilityMapObject,
    )

    __all__ = [
        "ReachabilityMapObject",
    ]
