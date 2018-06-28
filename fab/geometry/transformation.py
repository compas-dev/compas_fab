import math
from copy import deepcopy

# Support pre-release versions of compas
try:
    from compas.geometry.basic import multiply_matrix_vector
    from compas.geometry.basic import multiply_matrices
except ImportError:
    from compas.geometry.utilities import multiply_matrix_vector
    from compas.geometry.utilities import multiply_matrices

from compas.geometry import Transformation
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import Scale
from compas.geometry import Shear
from compas.geometry import Projection
from compas.geometry import matrix_from_translation
from compas.geometry import matrix_from_euler_angles
from compas.geometry import matrix_from_scale_factors
from compas.geometry import decompose_matrix
