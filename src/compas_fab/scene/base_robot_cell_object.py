from compas import IPY
from compas.scene import SceneObject

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_robots.scene import BaseRobotModelObject  # noqa: F401

        # from compas_fab.scene import BaseToolObject
        from compas_fab.robots import RigidBody  # noqa: F401
        from compas_fab.robots import RigidBodyState  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.scene import BaseRigidBodyObject  # noqa: F401


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


    """

    def __init__(self, draw_visual=True, draw_collision=False, scale=1.0, *args, **kwargs):
        super(BaseRobotCellObject, self).__init__(*args, **kwargs)
        self._draw_visual = draw_visual
        self._draw_collision = draw_collision
        self._scale = scale

        # Native Geometry handles

        self.robot_scene_object = SceneObject(self.robot_cell.robot_model)  # type: BaseRobotModelObject

        self.rigid_body_scene_objects = {}  # type: dict[str, BaseRigidBodyObject]
        for id, rigid_body in self.robot_cell.rigid_body_models.items():
            self.rigid_body_scene_objects[id] = SceneObject(rigid_body)

        self.tool_scene_objects = {}  # type: dict[str, BaseToolObject]
        for id, tool in self.robot_cell.tool_models.items():
            self.tool_scene_objects[id] = SceneObject(tool)

    @property
    def robot_cell(self):
        # type: () -> RobotCell
        return self.item

    # --------------------------------------------------------------------------
    # Draw Functions
    # --------------------------------------------------------------------------

    def draw(self, robot_cell_state=None):
        # type: (Optional[RobotCellState]) -> List[object]
        """Return all native geometry (in the CAD environment) belonging to the robot cell."""
        native_geometry = []
        if robot_cell_state:
            self.update(robot_cell_state)
        else:
            self.robot_scene_object.draw()
            for rigid_body_scene_object in self.rigid_body_scene_objects.values():
                rigid_body_scene_object.draw()
            for tool_scene_object in self.tool_scene_objects.values():
                tool_scene_object.draw()

    def draw_rigid_bodies(self, robot_cell_state=None):
        # type: (Optional[RobotCellState]) -> List[object]
        """Return the native geometry (in the CAD environment) of the rigid bodies of the robot cell."""
        native_geometry = []
        for id, rigid_body_scene_object in self.rigid_body_scene_objects.items():
            rigid_body_state = robot_cell_state.rigid_body_states[id] if robot_cell_state else None
            native_geometry += rigid_body_scene_object.draw(rigid_body_state)
        return native_geometry

    def update(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """Update the robot cell object with the given robot cell state."""
        # NOTE: All the constituent objects have an update method for transforming the native geometry
        if robot_cell_state.robot_configuration:
            self.robot_scene_object.update(robot_cell_state.robot_configuration)
        for id, rigid_body_state in robot_cell_state.rigid_body_states.items():
            self.rigid_body_scene_objects[id].update(rigid_body_state)
        for id, tool_state in robot_cell_state.tool_states.items():
            self.tool_scene_objects[id].update(tool_state)
