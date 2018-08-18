"""
********************************************************************************
compas_fab.geometry
********************************************************************************

.. currentmodule:: compas_fab.geometry

:mod:`compas_fab.geometry` contains classes and functions for
geometry representation and manipulation.


Classes
=======

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Frame
    Transformation
    Rotation
    Translation
    Scale
    Reflection
    Projection
    Shear

Transformations
===============

.. autosummary::
    :toctree: generated/
    :nosignatures:

    axis_and_angle_from_matrix
    axis_angle_vector_from_matrix
    basis_vectors_from_matrix
    compose_matrix
    decompose_matrix
    determinant
    euler_angles_from_matrix
    identity_matrix
    inverse
    matrix_from_axis_and_angle
    matrix_from_axis_angle_vector
    matrix_from_basis_vectors
    matrix_from_euler_angles
    matrix_from_frame
    matrix_from_perspective_entries
    matrix_from_quaternion
    matrix_from_scale_factors
    matrix_from_shear
    matrix_from_shear_entries
    matrix_from_translation
    quaternion_from_matrix
    translation_from_matrix
"""

from .frame import Frame
from .transformation import *
from .utilities import *
