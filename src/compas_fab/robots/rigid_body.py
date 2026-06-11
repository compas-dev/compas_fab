from typing import Optional
from typing import Union

from compas.data import Data
from compas.datastructures import Mesh

__all__ = [
    "RigidBody",
]


class RigidBody(Data):
    """Represents a rigid body."""

    def __init__(
        self,
        visual_meshes: Union[list[Mesh], Mesh],
        collision_meshes: Union[list[Mesh], Mesh],
        native_scale: float = 1.0,
        name: Optional[str] = None,
    ):
        """Represents a rigid body for use in a RobotCell.

        A rigid body can have different visual and collision meshes.

        The native unit (in compas_fab and backends) for the meshes is meters ``'native_scale=1.0'``.
        If the user created the rigid body in a different unit, the scale must be set such that the
        ``mesh.scale(native_scale)`` will convert the mesh to meters.
        For example, if the modeling environment is in millimeters, the scale should be set to 0.001.

        Notes
        -----
        The number of objects in the collision meshes does not have to be the same as the visual meshes.
        If the user wants to use the same mesh for both visualization and collision checking,
        place the meshes in visual_meshes and leave the collision_meshes empty.

        Parameters
        ----------
        visual_meshes
            The visual meshes of the rigid body used for visualization purpose.
            They can be more detailed for realistic visualization without affecting planning performance.
            If left empty, the rigid body will not be visualized.
        collision_meshes
            The collision meshes of the rigid body used for collision checking.
            They should be less detailed (fewer polygons) for better planning performance.
            If `None`, or an empty list is passed, no collision checking will be performed for the rigid body.
        native_scale
            The native scale factor of the meshes defined as `user_object_value * native_scale = meter_object_value`.
            In another words, `mesh.scale(native_scale)` will convert the input mesh to meters.
            For example, if the modeling environment is in millimeters, `native_scale` should be set to ``'0.001'``.
            Default is ``'1.0'``.
        name
            A human-readable identifier for the rigid body, stored via the base
            [`Data`][compas.data.Data] class.
            When the body is added to a [`RobotCell`][compas_fab.robots.RobotCell], this name is used as the
            key under which it is registered in ``cell.rigid_body_models``.
            Default is ``None`` (the inherited ``name`` then falls back to the class name).

        Attributes
        ----------
        visual_meshes
            A list of meshes for visualization purpose.
        collision_meshes
            A list of meshes for collision checking.

        Notes
        -----
        compas_fab do not support weight and inertia properties for rigid bodies.

        """
        super(RigidBody, self).__init__(name=name)

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

        self.native_scale = native_scale

    @property
    def __data__(self):
        return {
            "visual_meshes": self.visual_meshes,
            "collision_meshes": self.collision_meshes,
            "native_scale": self.native_scale,
        }

    @property
    def visual_meshes_in_meters(self) -> list[Mesh]:
        """The visual meshes of the rigid body in meters.

        This function returns a list of new meshes, the original meshes are not modified."""
        return [mesh.scaled(self.native_scale) for mesh in self.visual_meshes]

    @property
    def collision_meshes_in_meters(self) -> list[Mesh]:
        """The collision meshes of the rigid body in meters.

        This function returns a list of new meshes, the original meshes are not modified."""
        return [mesh.scaled(self.native_scale) for mesh in self.collision_meshes]

    @classmethod
    def from_mesh(cls, mesh: Mesh, native_scale: float = 1.0, name: Optional[str] = None) -> "RigidBody":
        """Creates a RigidBody from a single mesh.

        This function is a convenience function for creating a RigidBody from a single mesh.
        The mesh will be used for both visualization and collision checking.

        Parameters
        ----------
        mesh
            The mesh of the rigid body.
        native_scale
            The native scale of the rigid body. Default is 1.0.
        name
            A human-readable identifier for the rigid body. Default is ``None``.

        Returns
        -------
        [`RigidBody`][compas_fab.robots.RigidBody]
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls([mesh], [mesh], native_scale=native_scale, name=name)

    @classmethod
    def from_meshes(cls, meshes: list[Mesh], native_scale: float = 1.0, name: Optional[str] = None) -> "RigidBody":
        """Creates a RigidBody from a list of meshes.

        This function is a convenience function for creating a RigidBody from a list of meshes.
        The first mesh will be used for visualization and collision checking.
        The rest of the meshes will be used for visualization only.

        Parameters
        ----------
        meshes
            The meshes of the rigid body.
        native_scale
            The native scale of the rigid body. Default is 1.0.
        name
            A human-readable identifier for the rigid body. Default is ``None``.

        Returns
        -------
        [`RigidBody`][compas_fab.robots.RigidBody]
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls(meshes, meshes, native_scale=native_scale, name=name)
