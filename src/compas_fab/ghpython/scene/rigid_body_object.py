from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from compas_ghpython.drawing import draw_mesh
from .gh_scene_object import GHSceneObject
from compas_rhino.conversions import transformation_to_rhino


from compas_fab.scene import BaseRigidBodyObject
from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import List  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401
        from compas.geometry import Transformation  # noqa: F401


class RigidBodyObject(GHSceneObject, BaseRigidBodyObject):
    """Scene object for drawing a RigidBody in GHPython."""

    pass
