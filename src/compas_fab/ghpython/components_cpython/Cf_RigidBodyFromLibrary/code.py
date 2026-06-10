# r: compas_fab>=1.1.0
"""
Load a pre-defined RigidBody from RigidBodyLibrary.

Currently exposes `target_marker(size=1.0)`, useful as a visualization
aid for IK/planning targets in the scene.

The body's `.name` defaults to the library entry name (e.g. 'target_marker');
override it with `rigid_body_id` to control the key it is registered under when
wired into a Load Robot Cell component.

Also returns the body's visual meshes as Rhino geometry for preview, drawn in
the body's own base frame (not yet placed in any cell).

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas.scene import SceneObject
from compas_ghpython import error

from compas_fab.ghpython.scene import RigidBodyObject
from compas_fab.robots import RigidBodyLibrary


class RigidBodyFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, rigid_body_id: str, size: float):
        if not name:
            return (None, None)

        name = name.strip().lower()
        loader = getattr(RigidBodyLibrary, name, None)
        if loader is None:
            available = [attr for attr in dir(RigidBodyLibrary) if not attr.startswith("_") and callable(getattr(RigidBodyLibrary, attr))]
            error(ghenv.Component, "Unknown RigidBodyLibrary entry '{}'. Available: {}".format(name, ", ".join(available)))  # noqa: F821
            return (None, None)

        size = size if size else 1.0
        rigid_body = loader(size)
        rigid_body.name = rigid_body_id.strip() if rigid_body_id else name

        scene_object = SceneObject(
            item=rigid_body,
            sceneobject_type=RigidBodyObject,
            draw_visual=True,
            draw_collision=False,
        )
        meshes = scene_object.draw()

        return (rigid_body, meshes)
