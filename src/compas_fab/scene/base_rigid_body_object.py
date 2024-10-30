from compas.scene import SceneObject
from compas.geometry import Transformation
from compas.geometry import Scale

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import List  # noqa: F401
        from compas_fab.robots import RigidBody  # noqa: F401
        from compas_fab.robots import RigidBodyState  # noqa: F401


class BaseRigidBodyObject(SceneObject):
    """Base class for representing a rigid body object in a CAD environment.

    This class is automatically instantiated by the RobotCellObject when a RigidBody is present in the robot cell.
    However, it can also be used independently to draw a RigidBody in a CAD environment.

    Parameters
    ----------
    draw_visual : bool, optional
        If `True`, the visual meshes will be drawn. Default is `True`.
    draw_collision : bool, optional
        If `True`, the collision meshes will be drawn. Default is `False`.
    scale : float, optional
        The scale factor to visualize the rigid body, in case the native CAD environment
        uses a different unit system other than meters. The scale value should be set such
        that a `meter_mesh.scale(1/scale)` will create the mesh in the native unit system.
        For example, if the native unit system is millimeters, the scale should be set to `0.001`.
        Default is `1.0`.

    Notes
    -----
        The initial parameters `draw_visual`, `draw_collision`, and `scale` cannot be changed after initialization.

    """

    def __init__(self, draw_visual=True, draw_collision=False, scale=1.0, *args, **kwargs):
        super(BaseRigidBodyObject, self).__init__(*args, **kwargs)
        # These settings must not be changed after initialization
        self._draw_visual = draw_visual
        self._draw_collision = draw_collision
        self._scale = scale

        # These variables holds the native geometry
        # They will be filled when the `draw()` method is called for the first time.
        self._visual_mesh_native_geometry = []
        self._collision_mesh_native_geometry = []

        # The transformation of the object
        # This will be updated when the `update()` method is called. Do not edit this directly.
        self._mesh_current_transformation = Transformation()

    @property
    def rigid_body(self):
        # type: () -> RigidBody
        return self.item

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
        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if (not self._visual_mesh_native_geometry) and (not self._collision_mesh_native_geometry):
            self._initial_draw()

        # If a rigid body state is provided, update the object after drawing.
        # The update function will update the native geometry in place.
        if rigid_body_state:
            self.update(rigid_body_state)

        # Return the native geometry from the cache
        return self._visual_mesh_native_geometry + self._collision_mesh_native_geometry

    def update(self, rigid_body_state):
        # type: (RigidBodyState) -> None
        """Update the rigid body object with the given rigid body state.

        Parameters
        ----------
        rigid_body_state : :class:`~compas_fab.robots.RigidBodyState`
            The rigid body state to update the object.

        Notes
        -----
        Here we do not assume whether that the CAD-specific `._transform` method will operate on the native geometry
        in place, or creating a new object. This is up to the CAD-specific implementation.
        The `._transform` method should return the transformed geometry object, whether it is a new object or the same object.
        """

        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if (not self._visual_mesh_native_geometry) and (not self._collision_mesh_native_geometry):
            self._initial_draw()

        # The World Coordinate Frame (WCF) relative to the Visualization Coordinate Frame (VCF)
        t_vcf_wcf = Scale.from_factors([1 / self._scale] * 3)
        t_wcf_ocf = Transformation.from_frame(rigid_body_state.frame)

        new_transformation = t_vcf_wcf * t_wcf_ocf
        previous_transformation = self._mesh_current_transformation
        delta_transformation = new_transformation * previous_transformation.inverse()

        if self._draw_visual:
            new_native_geometry = [
                self._transform(geo, delta_transformation) for geo in self._visual_mesh_native_geometry
            ]
            # In case the _transform method returns a new object, we need to update the cache
            assert len(new_native_geometry) == len(self._visual_mesh_native_geometry)
            self._visual_mesh_native_geometry = new_native_geometry
        if self._draw_collision:
            new_native_geometry = [
                self._transform(geo, delta_transformation) for geo in self._collision_mesh_native_geometry
            ]
            # In case the _transform method returns a new object, we need to update the cache
            assert len(new_native_geometry) == len(self._collision_mesh_native_geometry)
            self._collision_mesh_native_geometry = new_native_geometry

        # Update the self._mesh_current_transformation
        self._mesh_current_transformation = new_transformation

    def _initial_draw(self):
        """Creating the native geometry when `draw()` or `update()` method is called for the first time.

        This private function is isolated out such that the initial draw does not happen in the __init__
        method and can be deferred until  `draw()` or `update()` is called for the first time.
        """
        # In the Rhino context, this allows the actual drawing to be deferred until the first time the object is drawn.

        self._visual_mesh_native_geometry = []
        self._collision_mesh_native_geometry = []
        if self._draw_visual:
            # NOTE: The meshes are initially drawn in meters scale and later transformed to
            #       the visualization scale in the `update()` method. This is to simplify
            #       the transformation logic in the `update()` method.
            for visual in self.rigid_body.visual_meshes_in_meters:
                name = self.rigid_body.name + "_visual"
                self._visual_mesh_native_geometry.append(self._create_geometry(visual, name=name))
        if self._draw_collision:
            for collision in self.rigid_body.collision_meshes_in_meters:
                name = self.rigid_body.name + "_collision"
                self._collision_mesh_native_geometry.append(self._create_geometry(collision, name=name))

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
