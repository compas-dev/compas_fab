"""Base scene object for compas_fab Rhino-context scene objects.

Provides the two CAD-specific hooks required by the cross-context base
classes in :mod:`compas_fab.scene`:

* :meth:`_create_geometry` — bake a compas :class:`~compas.datastructures.Mesh`
  into the active Rhino document (returns the resulting GUID).
* :meth:`_transform` — transform an already-baked Rhino doc object in place
  (returns the new GUID — Rhino's ``Objects.Transform(..., deleteOriginal=True)``
  always allocates a new GUID; the caller stores the returned GUID).

The cross-context base classes (:class:`~compas_fab.scene.BaseRobotModelObject`,
:class:`~compas_fab.scene.BaseRigidBodyObject`) compute *delta* transforms on
each ``update()`` call and apply them through ``_transform``, so an interactive
viewer can re-use the same baked geometry across many state updates without
re-baking the meshes.
"""

from typing import Optional

import scriptcontext as sc  # type: ignore
import System  # type: ignore
from compas.colors import Color
from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_rhino.conversions import mesh_to_rhino
from compas_rhino.conversions import transformation_to_rhino
from compas_rhino.scene import RhinoSceneObject as compas_rhino_RhinoSceneObject


def _is_identity(transformation: Transformation, tol: float = 1e-12) -> bool:
    """Cheap identity-check on a 4x4 :class:`compas.geometry.Transformation`."""
    matrix = transformation.matrix
    for i in range(4):
        for j in range(4):
            expected = 1.0 if i == j else 0.0
            if abs(matrix[i][j] - expected) > tol:
                return False
    return True


class RhinoSceneObject(compas_rhino_RhinoSceneObject):
    """Base for compas_fab Rhino scene objects with cached, in-place transforms.

    Subclasses combine this with one of the cross-context bases (e.g.
    :class:`~compas_fab.scene.BaseRobotModelObject`) so the cross-context
    base owns the cache and FK update logic, and this class supplies the
    Rhino-document-aware geometry operations.
    """

    def _create_geometry(
        self,
        geometry: Mesh,
        name: Optional[str] = None,
        color: Optional[Color] = None,
    ):
        """Bake a compas Mesh into the active Rhino document; return its GUID."""
        rhino_mesh = mesh_to_rhino(geometry)
        if rhino_mesh is None:
            return None
        if not rhino_mesh.IsValid:
            rhino_mesh.FillHoles()
        attributes = self.compile_attributes(name=name, color=color)
        guid = sc.doc.Objects.AddMesh(rhino_mesh, attributes)
        return guid

    def _transform(self, native_guid, transformation: Transformation):
        """Transform the doc object identified by ``native_guid`` in place.

        Rhino's ``Objects.Transform(guid, xform, deleteOriginal=True)`` always
        assigns a fresh GUID — the cross-context bases store the returned
        value back into their cache, so callers should treat the GUID as
        opaque between calls.

        Identity transforms are short-circuited to avoid allocating new GUIDs
        for every link on every frame (~30 no-op transforms per dual-arm
        update otherwise).
        """
        if native_guid is None:
            return None
        if _is_identity(transformation):
            return native_guid
        T = transformation_to_rhino(transformation)
        new_guid = sc.doc.Objects.Transform(native_guid, T, True)
        if new_guid == System.Guid.Empty:
            # Rhino refused the transform; keep the old GUID rather than orphaning the cache.
            return native_guid
        return new_guid
