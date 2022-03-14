"""
********************************************************************************
compas_fab.utilities
********************************************************************************

.. currentmodule:: compas_fab.utilities

Package containing a set of utility functions.

File system functions
=====================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    list_files_in_directory
    read_csv_to_dictionary
    read_data_from_pickle
    write_data_to_pickle

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

from .file_io import (
    read_csv_to_dictionary,
    read_data_from_json,
    read_data_from_pickle,
    write_data_to_json,
    write_data_to_pickle,
)
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
    # file_io
    'read_csv_to_dictionary',
    'read_data_from_json',
    'read_data_from_pickle',
    'write_data_to_json',
    'write_data_to_pickle',
    # filesystem
    'list_files_in_directory',
    # lazy_loader
    'LazyLoader',
    # numbers
    'allclose',
    'arange',
    'argmax',
    'argmin',
    'argsort',
    'clamp',
    'diffs',
    'map_range',
    'range_geometric_row',
    'sign',
]
