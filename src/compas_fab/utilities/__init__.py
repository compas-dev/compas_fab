"""
********************************************************************************
compas_fab.utilities
********************************************************************************

.. currentmodule:: compas_fab.utilities

Package containing a set of utility functions.


Numerical functions
===================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    allclose
    arange
    clamp
    diffs
    map_range
    range_geometric_row
    sign

Other functions
===============

.. autosummary::
    :toctree: generated/
    :nosignatures:

    argmax
    argmin
    argsort
    LazyLoader

"""

from .filesystem import (
    list_files_in_directory,
)
from .lazy_loader import (
    LazyLoader,
)
from .numbers import (
    allclose,
    arange,
    argmax,
    argmin,
    argsort,
    clamp,
    diffs,
    map_range,
    range_geometric_row,
    sign,
)

__all__ = [
    # filesystem
    "list_files_in_directory",
    # lazy_loader
    "LazyLoader",
    # numbers
    "allclose",
    "arange",
    "argmax",
    "argmin",
    "argsort",
    "clamp",
    "diffs",
    "map_range",
    "range_geometric_row",
    "sign",
]
