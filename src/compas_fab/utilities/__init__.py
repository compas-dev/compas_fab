"""Utility functions for compas_fab.

Numerical helpers (`allclose`, `arange`, `clamp`, `diffs`, `map_range`,
`range_geometric_row`, `sign`, `argmax`, `argmin`, `argsort`) and a
lazy-import helper (`LazyLoader`).
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
