from compas import IPY
from compas.data import Data

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import List  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401
__all__ = [
    "RigidBody",
]


class RigidBody(Data):
    """Represents a rigid body."""

    def __init__(self, visual_meshes, collision_meshes):
        # type: (List[Mesh] | Mesh, List[Mesh] | Mesh) -> None
        """Represents a rigid body.

        A rigid body can have different visual and collision meshes.

        Notes
        -----
        The number of objects in the collision meshes does not have to be the same as the visual meshes.
        If the user wants to use the same mesh for both visualization and collision checking,
        place the meshes in visual_meshes and leave the collision_meshes empty.

        Parameters
        ----------
        visual_meshes : list of :class:`compas.datastructures.Mesh` | :class:`compas.datastructures.Mesh`
            The visual meshes of the rigid body used for visualization purpose.
            They can be more detailed for realistic visualization without affecting planning performance.
        collision_meshes : list of :class:`compas.datastructures.Mesh` | :class:`compas.datastructures.Mesh`
            The collision meshes of the rigid body used for collision checking.
            They should be less detailed (fewer polygons) for better planning performance.
            If `None`, or an empty list is passed, no collision checking will be performed for the rigid body.

        Attributes
        ----------
        visual_meshes : list of :class:`compas.datastructures.Mesh`
            A list of meshes for visualization purpose.
        collision_meshes : list of :class:`compas.datastructures.Mesh`
            A list of meshes for collision checking.

        Notes
        -----
        compas_fab do not support weight and inertia properties for rigid bodies.

        """
        # type: (str, List[Mesh], List[Mesh]) -> None
        super(RigidBody, self).__init__()

        # If None is provided, we change that to an empty list
        if not visual_meshes:
            # If no input, set it to an empty list
            self.visual_meshes = []
        elif not isinstance(visual_meshes, list):
            # Ensure that it is a list
            self.visual_meshes = [visual_meshes]
        else:
            self.visual_meshes = visual_meshes

        if not collision_meshes:
            # If no input, set it to an empty list
            self.collision_meshes = []
        elif not isinstance(collision_meshes, list):
            # Ensure that it is a list
            self.collision_meshes = [collision_meshes]
        else:
            self.collision_meshes = collision_meshes

    @property
    def __data__(self):
        return {
            "visual_meshes": self.visual_meshes,
            "collision_meshes": self.collision_meshes,
        }

    @classmethod
    def from_mesh(cls, mesh):
        # type: (Mesh) -> RigidBody
        """Creates a RigidBody from a single mesh.

        This function is a convenience function for creating a RigidBody from a single mesh.
        The mesh will be used for both visualization and collision checking.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
            The mesh of the rigid body.

        Returns
        -------
        :class:`compas_fab.robots.RigidBody`
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls([mesh], [mesh])

    @classmethod
    def from_meshes(cls, meshes):
        # type: (List[Mesh]) -> RigidBody
        """Creates a RigidBody from a list of meshes.

        This function is a convenience function for creating a RigidBody from a list of meshes.
        The first mesh will be used for visualization and collision checking.
        The rest of the meshes will be used for visualization only.

        Parameters
        ----------
        meshes : list of :class:`compas.datastructures.Mesh`
            The meshes of the rigid body.

        Returns
        -------
        :class:`compas_fab.robots.RigidBody`
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls(meshes, meshes)
