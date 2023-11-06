from compas.data import Data
from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration, JointTrajectory  # noqa: F401

try:
    from compas_fab.planning import SceneState  # noqa: F401
except ImportError:
    from .state import SceneState  # noqa: F401

try:
    from typing import Optional  # noqa: F401
except ImportError:
    pass

__all__ = [
    "Action",
    "RoboticMovement",
    "LinearMovement",
    "FreeMovement",
    "OpenGripper",
    "CloseGripper",
]


class Action(Data):
    """Base class for all actions."""

    def __init__(self):
        super(Action, self).__init__()
        self.act_n = -1  # type: int
        self.tag = "Generic Action"

    @property
    def data(self):
        data = {}
        # For class inhereited from Action, use the following line
        # data = super(Action, self).data
        data["act_n"] = self.act_n
        data["tag"] = self.tag
        return data

    @data.setter
    def data(self, data):
        # For class inhereited from Action, use the following line
        # super(Action, type(self)).data.fset(self, data)
        self.act_n = data["act_n"]
        self.tag = data["tag"]

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.
        This method is called by the assembly process to apply the action to the scene state.
        """
        raise NotImplementedError


class RoboticMovement(Action):
    """Base class for all robotic movements.
    - Robotic movements are actions that changes the configuration of the robot.
    - Robotic movements require motion planning and are planned by a motion planner.

    Attributes
    ----------
    robot_target : :class:`compas.geometry.Frame`
        The target frame of the robot. In world coordinate frame.
    allowed_collision_pairs : list(tuple(str,str))
        List of pairs of collision objects that are allowed to collide.
    fixed_configuration : :class:`compas_fab.robots.Configuration`, optional
        The configuration of the robot if the target needs a fixed configuration.
        For example, if a taught position is used as a target.
    intermediate_planning_waypoint : list(:class:`compas_fab.robots.Configuration`), optional
        List of configurations that are used as waypoints during planning.
    planned_trajectory : :class:`compas_fab.robots.JointTrajectory`
        The planned trajectory of the robotic movement.
    planner_seed : int
        The random seed used by the planner to generate the trajectory.
    """

    def __init__(self):
        super(RoboticMovement, self).__init__()
        self.tag = "Generic Action"

        # Before Planning
        self.robot_target = None  # type: Frame
        self.allowed_collision_pairs = []  # type: list(tuple(str,str))
        self.fixed_configuration = None  # type: Optional[Configuration]
        self.intermediate_planning_waypoint = []  # type: list(Configuration)

        # After Planning
        self.planned_trajectory = None  # type: Optional[JointTrajectory]
        self.planner_seed = None  # type: Optional[int]

    @property
    def data(self):
        data = super(RoboticMovement, self).data
        data["tag"] = self.tag
        data["robot_target"] = self.robot_target
        data["allowed_collision_pairs"] = self.allowed_collision_pairs
        data["fixed_configuration"] = self.fixed_configuration
        data["intermediate_planning_waypoint"] = self.intermediate_planning_waypoint
        data["planned_trajectory"] = self.planned_trajectory
        data["planner_seed"] = self.planner_seed
        return data

    @data.setter
    def data(self, data):
        super(RoboticMovement, type(self)).data.fset(self, data)
        self.tag = data.get("tag", self.tag)
        self.robot_target = data.get("robot_target", self.robot_target)
        self.allowed_collision_pairs = data.get("allowed_collision_pairs", self.allowed_collision_pairs)
        self.fixed_configuration = data.get("fixed_configuration", self.fixed_configuration)
        self.intermediate_planning_waypoint = data.get(
            "intermediate_planning_waypoint", self.intermediate_planning_waypoint
        )
        self.planned_trajectory = data.get("planned_trajectory", self.planned_trajectory)
        self.planner_seed = data.get("planner_seed", self.planner_seed)

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is updated with the new robot state.
        If a trajectory is available, the robot state is updated with the last configuration of the trajectory.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        robot_state = scene_state.get_robot_state()
        if self.planned_trajectory is not None:
            robot_state.configuration = self.planned_trajectory.points[-1]
        elif self.fixed_configuration is not None:
            robot_state.configuration = self.fixed_configuration
        robot_state.frame = self.robot_target
        if debug:
            print("Robot Moved.")

        # Transform attached objects
        attached_tool_id = scene_state.get_attached_tool_id()
        if attached_tool_id is not None:
            attached_tool_state = scene_state.get_tool_state(attached_tool_id)
            assert attached_tool_state.attached_to_robot, "Inconsistency: Attached tool must be attached to robot."
            # attached_tool_state.frame = robot_state.frame * attached_tool_state.attached_to_robot_grasp
            attached_tool_state.frame = Frame.from_transformation(
                Transformation.from_frame(robot_state.frame) * attached_tool_state.attached_to_robot_grasp
            )
            if debug:
                print("- Attached Tool %s Followed." % attached_tool_id)
        # Transform attached workpieces
        attached_workpiece_id = scene_state.get_attached_workpiece_id()
        if attached_workpiece_id is not None:
            attached_workpiece = scene_state.get_workpiece_state(attached_workpiece_id)
            assert (
                attached_workpiece.attached_to_tool_id == attached_tool_id
            ), "Inconsistency: Attached workpiece must be attached to the attached tool."
            attached_workpiece.frame = Frame.from_transformation(
                Transformation.from_frame(attached_tool_state.frame) * attached_workpiece.attached_to_tool_grasp
            )
            if debug:
                print("- Attached Workpiece %s Followed." % attached_workpiece_id)


class LinearMovement(RoboticMovement):
    """Base class for all linear robotic movements.
    Linear robotic movements are planned by Linear Motion Planners.
    """


class FreeMovement(RoboticMovement):
    """Base class for all free robotic movements.
    Free robotic movements are planned by Free Motion Planners.
    """


class OpenGripper(Action):
    """Action to open the gripper.
    If the gripper is closed around a workpiece, the workpiece is detached from the gripper.
    It is possible to open the gripper with or without a workpiece attached.
    """

    def __init__(self):
        super(OpenGripper, self).__init__()

    @property
    def data(self):
        data = super(OpenGripper, self).data
        return data

    @data.setter
    def data(self, data):
        super(OpenGripper, type(self)).data.fset(self, data)

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is updated with the new robot state.
        It is possible to open the gripper with or without a workpiece attached.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        # Transform attached objects
        attached_tool_id = scene_state.get_attached_tool_id()
        assert attached_tool_id is not None, "Inconsistency: No tool attached to robot."
        # attached_tool_state = scene_state.get_tool_state(attached_tool_id)
        # TODO: Check if tool is a gripper
        # TODO: Change the configuration of the gripper to opened

        # Transform attached workpieces
        attached_workpiece_id = scene_state.get_attached_workpiece_id()
        if debug:
            print("Gripper opened.")
        if attached_workpiece_id is not None:
            attached_workpiece = scene_state.get_workpiece_state(attached_workpiece_id)
            attached_workpiece.attached_to_tool_id = None
            attached_workpiece.attached_to_tool_grasp = None
            if debug:
                print("- Workpiece %s detached from tool." % attached_workpiece_id)


class CloseGripper(Action):
    """Action to close the gripper.

    If the gripper is closed around a workpiece, the workpiece is attached to the gripper.

    Attributes
    ----------
    tool_id : str
        The id of the gripper tool that is used.
    attached_workpiece_id : str, optional
        The id of the workpiece attached to the gripper.
        If the workpiece is not specified, the gripper is assumed to be empty.
    attached_workpiece_grasp : :class:`compas.geometry.Transformation`, optional
        The grasp frame of the workpiece relative to the robot flange.
        If the workpiece_frame is not specified, the workpiece Frame is
        assumed to be the same as the gripper's TCP.
    """

    def __init__(self):
        super(CloseGripper, self).__init__()
        self.tool_id = None  # type: Frame
        self.attached_workpiece_id = None  # type: Frame
        self.attached_workpiece_grasp = None  # type: Frame

    @property
    def data(self):
        data = super(CloseGripper, self).data
        data["attached_workpiece_id"] = self.attached_workpiece_id
        data["attached_workpiece_grasp"] = self.attached_workpiece_grasp
        return data

    @data.setter
    def data(self, data):
        super(CloseGripper, type(self)).data.fset(self, data)
        self.attached_workpiece_id = data.get("attached_workpiece_id", self.attached_workpiece_id)
        self.attached_workpiece_grasp = data.get("attached_workpiece_grasp", self.attached_workpiece_grasp)

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is updated with the new robot state.
        It is possible to open the gripper with or without a workpiece attached.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        # Transform attached objects
        attached_tool_id = scene_state.get_attached_tool_id()
        assert attached_tool_id is not None, "Inconsistency: No tool attached to robot."
        attached_tool_state = scene_state.get_tool_state(attached_tool_id)
        # TODO: Check if tool is a gripper
        # TODO: Change the configuration of the gripper to closed

        if debug:
            print("Gripper closed.")
        # Transform attached workpieces
        if self.attached_workpiece_id is not None:
            existing_workpiece_id = scene_state.get_attached_workpiece_id()
            assert existing_workpiece_id is None, "Inconsistency: Another workpiece is already attached to the tool."
            attached_workpiece = scene_state.get_workpiece_state(self.attached_workpiece_id)
            # Update the workpiece grasp and frame
            attached_workpiece.attached_to_tool_id = attached_tool_id
            attached_workpiece.attached_to_tool_grasp = self.attached_workpiece_grasp or Transformation()
            if debug:
                print("- Workpiece %s attached to tool." % self.attached_workpiece_id)
            # attached_workpiece.frame = attached_tool_state.frame * attached_workpiece.attached_to_tool_grasp
            attached_workpiece.frame = Frame.from_transformation(
                Transformation.from_frame(attached_tool_state.frame) * attached_workpiece.attached_to_tool_grasp
            )


class LoadWorkpiece(Action):
    """Action to load a workpiece, probably performed manually.
    This moves the workpiece to a specific frame.

    Attributes
    ----------
    workpiece_id : str
        The id of the workpiece to be loaded.
    """

    def __init__(self):
        super(LoadWorkpiece, self).__init__()
        self.workpiece_id = None  # type: str
        self.frame = None  # type: Frame

    @property
    def data(self):
        data = super(LoadWorkpiece, self).data
        data["workpiece_id"] = self.workpiece_id
        data["frame"] = self.frame
        return data

    @data.setter
    def data(self, data):
        super(LoadWorkpiece, type(self)).data.fset(self, data)
        self.workpiece_id = data.get("workpiece_id", self.workpiece_id)
        self.frame = data.get("frame", self.frame)

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.
        The SceneState is updated with the new robot state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        # Transform attached objects
        workpiece_state = scene_state.get_workpiece_state(self.workpiece_id)
        assert workpiece_state.attached_to_tool_id is None, "Inconsistency: Workpiece is already attached to a tool."
        workpiece_state.frame = self.frame
        if debug:
            print("Workpiece %s loaded to new location." % self.workpiece_id)
