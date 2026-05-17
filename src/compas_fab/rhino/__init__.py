"""Rhino scene objects for compas_fab.

Scene objects expose `compas_fab` data structures to Rhino while keeping
the data layer separated from CAD-specific code and leveraging native
Rhino performance.
"""

import compas

if compas.RHINO:
    from .scene import (
        ReachabilityMapObject,
    )

    __all__ = [
        "ReachabilityMapObject",
    ]
