# r: compas_fab>=1.1.0
"""
Load a pre-defined RigidBody from RigidBodyLibrary.

Currently exposes `target_marker(size=1.0)`, useful as a visualization
aid for IK/planning targets in the scene.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.robots import RigidBodyLibrary


class RigidBodyFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, size: float):
        if not name:
            return None

        name = name.strip().lower()
        loader = getattr(RigidBodyLibrary, name, None)
        if loader is None:
            available = [attr for attr in dir(RigidBodyLibrary) if not attr.startswith("_") and callable(getattr(RigidBodyLibrary, attr))]
            error(ghenv.Component, "Unknown RigidBodyLibrary entry '{}'. Available: {}".format(name, ", ".join(available)))  # noqa: F821
            return None

        size = size if size else 1.0
        return loader(size)
