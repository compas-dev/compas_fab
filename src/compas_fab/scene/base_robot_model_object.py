from compas import IPY
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.scene import SceneObject
from compas_robots.model import MeshDescriptor

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_robots import Configuration  # noqa: F401
        from compas_robots import RobotModel  # noqa: F401
        from compas_robots.model import Collision  # noqa: F401
        from compas_robots.model import Link  # noqa: F401
        from compas_robots.model import Visual  # noqa: F401


class BaseRobotModelObject(SceneObject):
    """Base class for compas_fab-special RobotModelObjects" """

    MESH_JOIN_PRECISION = 12

    def __init__(self, draw_visual=True, draw_collision=False, scale=1.0, *args, **kwargs):
        # type: (Optional[bool], Optional[bool], Optional[float], *object, **object) -> None
        super(BaseRobotModelObject, self).__init__(*args, **kwargs)
        self._draw_visual = draw_visual
        self._draw_collision = draw_collision
        self._scale = scale

        # It will be filled when the `draw()` method is called for the first time.
        self.links_visual_mesh_native_geometry = {}
        self.links_collision_mesh_native_geometry = {}
        # Dictionary to hold the current transformation of the robot links
        self.links_visual_mesh_transformation = {}
        self.links_collision_mesh_transformation = {}

    @property
    def robot_model(self):
        # type: () -> RobotModel
        """The robot model this object is associated with."""
        return self.item

    def draw(self, robot_configuration=None, base_frame=None):
        # type: (Optional[Configuration], Optional[Frame]) -> List[object]
        """Draw the robot model object in the respective CAD environment.

        This function conforms to `SceneObject.draw()` Interface and will return
        the native geometry objects that were created.

        - In the Rhino context where the geometry had been placed in the Rhino document,
          the native geometry handles are still returned.
        - In the GHPython context, the native geometry is returned as a list of
          `Rhino.Geometry.Mesh` objects, which can be used as a GHComponent output.

        """
        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if (not self.links_visual_mesh_native_geometry) and (not self.links_collision_mesh_native_geometry):
            self._initial_draw()

        # Update the robot model if a configuration is provided
        if robot_configuration:
            self.update(robot_configuration, base_frame)

        # Return the native geometry
        native_geometry = []
        if self._draw_visual:
            native_geometry.extend(self.links_visual_mesh_native_geometry.values())
        if self._draw_collision:
            native_geometry.extend(self.links_collision_mesh_native_geometry.values())
        return native_geometry

    def _initial_draw(self):
        """Creating the native geometry when `draw()` method is called for the first time.

        This private function is isolated out such that the initial draw does not happen in the __init__
        method and can be deferred until the `draw()` method is called for the first time.
        """
        # Reset the dictionaries
        self.base_native_geometry = None
        self.base_transformation = None
        self.links_visual_mesh_native_geometry = {}  # type: dict[str, Mesh]
        self.links_collision_mesh_native_geometry = {}  # type: dict[str, Mesh]
        self.links_visual_mesh_transformation = {}  # type: dict[str, Transformation]
        self.links_collision_mesh_transformation = {}  # type: dict[str, Transformation]

        # Iterate over the links and create the visual and collision meshes.
        # Only create the geometry if it exists.
        for link in self.robot_model.iter_links():
            link_name = link.name
            # CREATE VISUAL MESHES
            if self._draw_visual:
                visual_mesh = self.robot_model.get_link_visual_meshes_joined(link)
                if visual_mesh:
                    self.links_visual_mesh_native_geometry[link_name] = self._create_geometry(
                        visual_mesh, name=link_name + "_visual"
                    )
                    self.links_visual_mesh_transformation[link_name] = Transformation()
            # CREATE COLLISION MESHES
            if self._draw_collision:
                collision_mesh = self.robot_model.get_link_collision_meshes_joined(link)
                if collision_mesh:
                    self.links_collision_mesh_native_geometry[link_name] = self._create_geometry(
                        collision_mesh, name=link_name + "_collision"
                    )
                    self.links_collision_mesh_transformation[link_name] = Transformation()

    def update(self, robot_configuration, base_frame=None):
        # type: (Configuration, Optional[Frame]) -> None
        """Updates the native geometry of the robot model according to the given robot
        configuration and base frame.

        Parameters
        ----------
        robot_configuration : :class:`~compas_robots.Configuration`
            The robot configuration to update the robot model.
        base_frame : :class:`~compas.geometry.Frame`, optional
            The frame of the RobotCoordinateFrame relative to the WorldCoordinateFrame.
            Default is World XY frame at origin.

        Returns
        -------
        None

        Notes
        -----
        It is possible to call this method before the `draw()` method, but the native geometry

        """

        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if (not self.links_visual_mesh_native_geometry) and (not self.links_collision_mesh_native_geometry):
            self._initial_draw()

        if len(self.robot_model.get_configurable_joints()) == 0:
            return

        def _update_link_meshes(link_name, new_transformation):
            if link_name in self.links_visual_mesh_native_geometry:
                # Compute the delta transformation
                previous_transformation = self.links_visual_mesh_transformation[link_name]
                delta_transformation = new_transformation * previous_transformation.inverse()  # type: Transformation
                # Transform the native geometry
                native_geometry = self.links_visual_mesh_native_geometry[link_name]
                new_native_geometry = self._transform(native_geometry, delta_transformation)
                # Update the dictionaries
                self.links_visual_mesh_native_geometry[link_name] = new_native_geometry
                self.links_visual_mesh_transformation[link_name] = new_transformation
                print(
                    "Updated visual link '{}' from previous '{}' with delta '{}' to new transformation '{}'.".format(
                        link_name, previous_transformation, delta_transformation, new_transformation
                    )
                )
            if link_name in self.links_collision_mesh_native_geometry:
                # Compute the delta transformation
                previous_transformation = self.links_collision_mesh_transformation[link_name]
                delta_transformation = new_transformation * previous_transformation.inverse()
                # Transform the native geometry
                native_geometry = self.links_collision_mesh_native_geometry[link_name]
                new_native_geometry = self._transform(native_geometry, delta_transformation)
                # Update the dictionaries
                self.links_collision_mesh_native_geometry[link_name] = new_native_geometry
                self.links_collision_mesh_transformation[link_name] = new_transformation

        # Use the robot base frame if it is provided
        t_wcf_rcf = Transformation.from_frame(base_frame) if base_frame else Transformation()

        # Update the base link
        _update_link_meshes(self.robot_model.root.name, t_wcf_rcf)

        # Iterate over the joints (equivalent to all the child links) and update their pose
        # This iteration order is the same as the result from transform_frames()
        transformed_joint_frames = self.robot_model.transformed_frames(robot_configuration)
        for joint_frame, joint in zip(transformed_joint_frames, list(self.robot_model.iter_joints())):
            # From Robot Coordinate Frame to Link Coordinate Frame (Link Frame == Joint Frame)
            t_rcf_lcf = Transformation.from_frame(joint_frame)
            # From World Coordinate Frame to Link Coordinate Frame
            t_wcf_lcf = t_wcf_rcf * t_rcf_lcf
            _update_link_meshes(joint.child_link.name, t_wcf_lcf)

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
