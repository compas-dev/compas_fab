from compas import IPY
from compas.scene import SceneObject


if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.scene import BaseRigidBodyObject  # noqa: F401

        from .base_robot_model_object import BaseRobotModelObject  # noqa: F401


class BaseRobotCellObject(SceneObject):
    """Base class for all robot cell objects.

    The RobotCellObject is a container for all the objects that are part of a robot cell.
    It does not draw and update the native geometry itself, but delegates this to the
    respective SceneObject instances that are part of the robot cell.

    The RobotCellObject and its constituent CAD-specific SceneObject instances provides
    caching of the native geometry whenever possible, so that the native geometry is only
    created once and then updated in place. This provides a performance improvement when
    the robot cell is drawn multiple times such as during interactive manipulation (e.g.
    in GHPython) or visualizing a trajectory as animation.

    In order to take advantage of the the caching, users should reuse the RobotCellObject
    instance as long as the robot cell remain the same, and only update the robot cell state when needed.

    If caching function is not desired, simply remove the native geometry and create
    a new instance of the RobotCellObject every time the robot cell is drawn.

    Parameters
    ----------
    draw_visual : bool, optional
        If `True`, the visual meshes will be drawn. Default is `True`.
    draw_collision : bool, optional
        If `True`, the collision meshes will be drawn. Default is `False`.
    native_scale : float, optional
        The native scale factor to visualize the meshes in the robot cell, in case the native CAD environment
        uses a different unit system other than meters. The native scale value should be set such
        that a `meter_mesh.scale(1/native_scale)` will create the mesh in the native unit system.
        For example, if the unit system of the visualization environment  is millimeters, `native_scale` should be set to ``'0.001'``.
        Default is `1.0`.

    Notes
    -----
        The initial parameters `draw_visual`, `draw_collision`, and `scale` cannot be changed after initialization.


    """

    def __init__(self, draw_visual=True, draw_collision=False, native_scale=1.0, *args, **kwargs):
        super(BaseRobotCellObject, self).__init__(*args, **kwargs)
        self._draw_visual = draw_visual
        self._draw_collision = draw_collision
        self._native_scale = native_scale

        # Native Geometry handles
        # robot_model_object = self._get_robot_model_object()
        self._robot_model_scene_object = None  # type: BaseRobotModelObject
        self._rigid_body_scene_objects = {}  # type: dict[str, BaseRigidBodyObject]
        self._tool_scene_objects = {}  # type: dict[str, BaseRobotModelObject]

    @property
    def robot_cell(self):
        # type: () -> RobotCell
        return self.item

    # --------------------------------------------------------------------------
    # Draw Functions
    # --------------------------------------------------------------------------

    def draw(self, robot_cell_state=None):
        # type: (Optional[RobotCellState]) -> List[object]
        """Return all native geometry (in the CAD environment) belonging to the robot cell.

        The geometry contains the robot model, rigid bodies, and tools.

        Parameters
        ----------
        robot_cell_state : RobotCellState, optional
            The state of the robot cell, containing the configuration of the robot and the state of the rigid bodies and tools.

        Returns
        -------
        List[object]
            A list of native geometry objects representing the robot cell in the CAD environment.
        """
        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if not self._robot_model_scene_object:
            self._initial_draw()

        # Update the child scene objects if a configuration is provided
        if robot_cell_state:
            self.update(robot_cell_state)

        # The native geometry are collected from the child scene objects .draw() method.
        native_geometries = []
        native_geometries.extend(self._robot_model_scene_object.draw())

        for rigid_body_scene_object in self._rigid_body_scene_objects.values():
            native_geometries.extend(rigid_body_scene_object.draw())
        for tool_scene_object in self._tool_scene_objects.values():
            native_geometries.extend(tool_scene_object.draw())

        return native_geometries

    def update(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """Update the robot cell object with the given robot cell state."""
        # NOTE: All the constituent objects have an update method for transforming the native geometry

        # Create the native geometry when the `draw()` or `update()` method is called for the first time
        if not self._robot_model_scene_object:
            self._initial_draw()

        # Compute the attach objects frames
        robot_cell_state = self.robot_cell.compute_attach_objects_frames(robot_cell_state)

        # Update the child scene objects if a configuration is provided
        if robot_cell_state:
            self._robot_model_scene_object.update(
                robot_cell_state.robot_configuration, robot_cell_state.robot_base_frame
            )
            for id, rigid_body_scene_object in self._rigid_body_scene_objects.items():
                rigid_body_state = robot_cell_state.rigid_body_states[id]
                rigid_body_scene_object.update(rigid_body_state)
            # Tools are drawn by RobotModelObject, so we need to set their base frame with tool frame
            for id, tool_scene_object in self._tool_scene_objects.items():
                tool_state = robot_cell_state.tool_states[id]
                tool_scene_object.update(tool_state.configuration, tool_state.frame)

    # --------------------------------------------------------------------------

    def _initial_draw(self):
        """Creating the native geometry when `draw()` or `update()` method is called for the first time.

        This private function is isolated out such that the initial draw does not happen in the __init__
        method and can be deferred until  `draw()` or `update()` is called for the first time.
        """

        raise NotImplementedError
