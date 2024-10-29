from compas.scene import SceneObject
from compas.geometry import Transformation

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import List  # noqa: F401
        from compas_fab.robots import RigidBody  # noqa: F401
        from compas_fab.robots import RigidBodyState  # noqa: F401


class BaseRigidBodyObject(SceneObject):

    def __init__(self, draw_visual=True, draw_collision=False, scale=1.0, *args, **kwargs):
        super(BaseRigidBodyObject, self).__init__(*args, **kwargs)
        # These settings must not be changed after initialization
        self._draw_visual = draw_visual
        self._draw_collision = draw_collision
        self._scale = scale

        # These variables holds the native geometry
        # They will be filled when the `draw()` method is called for the first time.
        self.visual_mesh_native_geometry = []
        self.collision_mesh_native_geometry = []

    # This implements the SceneObject.draw method
    def draw(self, rigid_body_state=None):
        # type: (Optional[RigidBodyState]) -> List[object]
        """Draw the rigid body object in the respective CAD environment.
        It will return the native geometry objects that were created.

        In Rhino, this method will create Rhino.Geometry.Mesh objects and add them to the Rhino document.
        In GHPython, this method will create Rhino.Geometry.Mesh objects and return them.

        Parameters
        ----------
        rigid_body_state : :class:`~compas_fab.robots.RigidBodyState`, optional
            The rigid body state to draw.

        Returns
        -------
        List[object]
            A list of native geometry objects representing the visual and collision meshes of the rigid body.
        """
        # The native geometry is created when the `draw()` method is called for the first time
        # In the Rhino context, this allows the actual drawing to be deferred until the first time the object is drawn.
        if (not self.visual_mesh_native_geometry) and (not self.collision_mesh_native_geometry):
            self.visual_mesh_native_geometry = []
            self.collision_mesh_native_geometry = []
            if self._draw_visual:
                for visual in self.rigid_body.visual_meshes_in_meters:
                    name = self.rigid_body.name + "_visual"
                    self.visual_mesh_native_geometry.append(self._create_geometry(visual, name=name))
            if self._draw_collision:
                for collision in self.rigid_body.collision_meshes_in_meters:
                    name = self.rigid_body.name + "_collision"
                    self.collision_mesh_native_geometry.append(self._create_geometry(collision, name=name))

        # If a rigid body state is provided, update the object after drawing.
        # The update function will update the native geometry in place.
        if rigid_body_state:
            self.update(rigid_body_state)

        # Return the native geometry from the cache
        return self.visual_mesh_native_geometry + self.collision_mesh_native_geometry

    # This is called by the BaseRobotCellObject to update the rigid body object
    def update(self, rigid_body_state):
        # type: (RigidBodyState) -> None
        """Update the rigid body object with the given rigid body state.

        Here we do not assume whether that the CAD-specific `._transform` method will operate on the native geometry
        in place, or creating a new object. This is up to the CAD-specific implementation.
        The `._transform` method should return the transformed geometry object, whether it is a new object or the same object.
        """
        previous_transformation = self.transformation or Transformation()
        new_transformation = Transformation.from_frame(rigid_body_state.frame)
        delta_transformation = new_transformation * previous_transformation.inverse()

        if self._draw_visual:
            visual_mesh_native_geometry = [
                self._transform(geo, delta_transformation) for geo in self.visual_mesh_native_geometry
            ]
            # In case the _transform method returns a new object, we need to update the cache
            assert len(visual_mesh_native_geometry) == len(self.visual_mesh_native_geometry)
            self.visual_mesh_native_geometry = visual_mesh_native_geometry
        if self._draw_collision:
            collision_mesh_native_geometry = [
                self._transform(geo, delta_transformation) for geo in self.collision_mesh_native_geometry
            ]
            # In case the _transform method returns a new object, we need to update the cache
            assert len(visual_mesh_native_geometry) == len(self.visual_mesh_native_geometry)
            self.collision_mesh_native_geometry = collision_mesh_native_geometry

        # Update the self.transformation
        self.transformation = new_transformation

    @property
    def rigid_body(self):
        # type: () -> RigidBody
        return self.item

    # --------------------------------------------------------------------------
    # Private methods that need to be implemented by CAD specific classes
    # --------------------------------------------------------------------------

    def _transform(self, geometry, transformation):
        """Transforms the given native CAD-specific geometry.

        Here we do not assume whether that the CAD-specific `._transform` method will operate on the native geometry
        in place, or creating a new object. This is up to the CAD-specific implementation.
        The `._transform` method should return the transformed geometry object, whether it is a new object or the same object.

        Parameters
        ----------
        geometry : object
            A CAD-specific (i.e. native) geometry object as returned by :meth:`_create_geometry`.
        transformation : :class:`~compas.geometry.Transformation`
            Transformation to update the geometry object.

        Returns
        -------
        object
            The transformed geometry object.

        """
        raise NotImplementedError

    def _create_geometry(self, geometry, name=None, color=None):
        """Create new native geometry in the respective CAD environment.

        Parameters
        ----------
        geometry : :class:`~compas.datastructures.Mesh`
            Instance of a mesh data structure
        name : str, optional
            The name of the mesh to draw.
        color : :class:`~compas.colors.Color`
            The color of the object.`

        Returns
        -------
        object
            CAD-specific geometry

        Notes
        -----
        This is an abstract method that needs to be implemented by derived classes.

        """
        raise NotImplementedError
