from compas.data import Data
from compas.geometry import Point
from compas.geometry import Frame
from compas.geometry import Transformation

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
    """Base class for all actions.

    The Action Classes are intended to be used to model the actions of a human operator or a robot.
    Current Implementation assumes discrete actions that are executed sequentially, aka. one after another.

    Actions can be created automatically or manually. They are stored in a :class:`compas_fab.planning.Processs`.
    Some actions contain references to workpieces, tools, or other objects in the scene. Those objects are
    stored in dictionaries under the Process object and are identified by their id.

    Actions are closely related to the :class:`compas_fab.planning.SceneState` because each action changes the scene state.
    Provided with an starting scene state, the actions can be *applied to* create the ending scene state.
    The only exception are robotic configurations that are changed by :class:`compas_fab.planning.RoboticMovement` actions.
    Those actions changes the robot configuration and their effect is not known until after the planning process.

    Attributes
    ----------
    act_id : [str | int]
        A unique id of the action, it is optional but recommended for debugging.
    tag : str
        A human readable tag that describes the action. It is printed out during visualization, debugging, execution and logging.
    """

    def __init__(self):
        super(Action, self).__init__()
        self.act_id = -1  # type: [str | int]
        self.tag = "Generic Action"  # type: str

    @property
    def data(self):
        data = {}
        # For class inhereited from Action, use the following line
        # data = super(Action, self).data
        data["act_id"] = self.act_id
        data["tag"] = self.tag
        return data

    @data.setter
    def data(self, data):
        # For class inhereited from Action, use the following line
        # super(Action, type(self)).data.fset(self, data)
        self.act_id = data["act_id"]
        self.tag = data["tag"]

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.
        This method is called by the assembly process to apply the action to the scene state.
        """
        raise NotImplementedError("Action.apply_to() is not implemented by %s." % type(self))


class RoboticMovement(Action):
    """Base class for all robotic movements.

    Robotic movements are actions that changes the robot configuration and
    hence also the frame of the robot flange and all attached tools and workpieces.

    The RoboticMovement class only describes the target (ending state)
    of the robotic movement, whereas the starting state is defined using a
    :class:`compas_fab.planning.SceneState` object. The starting state also
    defines the attached tools and workpieces.
    Both objects are required by the motion planner to plan a trajectory.

    After motion planning, the trajectory can be stored in the same RoboticMovement class
    and can be used for visualization and execution.

    When applied to a scene state, Robotic movements also changes the state of the
    attached tool and workpiece. If the trajectory have been planned, the configuration
    of the robot is updated to the last configuration of the trajectory. See
    :meth:`compas_fab.planning.RoboticMovement.apply_to` for more details.

    Attributes (Before Planning)
    ----------------------------
    target_robot_flange_frame : :class:`compas.geometry.Frame`
        The robot flange frame of the robot at target. In world coordinate frame.
    allowed_collision_pairs : list(tuple(str,str))
        List of pairs of collision objects that are allowed to collide.
        Objects are identified by workpiece_id, tool_id, or static_object_id.
    fixed_target_configuration : :class:`compas.robots.Configuration`, optional
        The configuration of the robot if the target needs a fixed configuration.
        For example, if a taught position is used as a target.
    fixed_trajectory : :class:`compas_fab.robots.JointTrajectory`, optional
        The trajectory of the robot if the trajectory is fixed.
        For example, if a pre-planned or recorded trajectory is preferred for this action.
        Specifying a fixed trajectory will force the chained motion planner to respect this trajectory
        when planning neighbouring robotic movements. This can be used to improve repeatibility.
        Users must be careful to specify a trajectory that is collision free and does not violate
        the kinematic limits of the robot.
    tag : str
        A human readable tag that describes the action.
        It can printed out during visualization, debugging, execution and logging.

    Attributes (After Planning)
    ---------------------------
    planned_trajectory : :class:`compas_fab.robots.JointTrajectory`
        The planned trajectory of the robotic movement. Available after planning. The first and last
        trajectory points corresponds to the starting and ending configuration of the robotic movement.
    planner_seed : int
        The random seed used by the planner to generate the trajectory. Used by some planners.

    Attributes (For Execution)
    --------------------------
    speed_data_id : str
        The id (or name) of the speed data to be used for execution.
    ----------

    """

    def __init__(self):
        super(RoboticMovement, self).__init__()
        self.tag = "Generic Action"

        # Before Planning
        self.target_robot_flange_frame = None  # type: Frame
        self.allowed_collision_pairs = []  # type: list(tuple(str,str))
        self.fixed_target_configuration = None  # type: Optional[Configuration]
        self.fixed_trajectory = None  # type: Optional[JointTrajectory]
        self.intermediate_planning_waypoint = []  # type: list(Configuration)

        # After Planning
        self.planned_trajectory = None  # type: Optional[JointTrajectory]
        self.planner_seed = None  # type: Optional[int]

        # For Execution
        self.speed_data_id = None  # type: Optional[str]

    @property
    def data(self):
        data = super(RoboticMovement, self).data
        data["tag"] = self.tag
        data["target_robot_flange_frame"] = self.target_robot_flange_frame
        data["allowed_collision_pairs"] = self.allowed_collision_pairs
        data["fixed_target_configuration"] = self.fixed_target_configuration
        data["fixed_trajectory"] = self.fixed_trajectory
        data["intermediate_planning_waypoint"] = self.intermediate_planning_waypoint
        data["planned_trajectory"] = self.planned_trajectory
        data["planner_seed"] = self.planner_seed
        data["speed_data_id"] = self.speed_data_id
        return data

    @data.setter
    def data(self, data):
        super(RoboticMovement, type(self)).data.fset(self, data)
        self.tag = data.get("tag", self.tag)
        self.target_robot_flange_frame = data.get("target_robot_flange_frame", self.target_robot_flange_frame)
        self.allowed_collision_pairs = data.get("allowed_collision_pairs", self.allowed_collision_pairs)
        self.fixed_target_configuration = data.get("fixed_target_configuration", self.fixed_target_configuration)
        self.fixed_trajectory = data.get("fixed_trajectory", self.fixed_trajectory)
        self.intermediate_planning_waypoint = data.get(
            "intermediate_planning_waypoint", self.intermediate_planning_waypoint
        )
        self.planned_trajectory = data.get("planned_trajectory", self.planned_trajectory)
        self.planner_seed = data.get("planner_seed", self.planner_seed)
        self.speed_data_id = data.get("speed_data_id", self.speed_data_id)

    def apply_to(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is updated with the new robot state.
        If a fixed_trajectory or planned_trajectory is available, the robot_state.configuration is updated with the last configuration of the trajectory.
        If fixed_target_configuration is available, the robot_state.configuration is updated with the fixed_target_configuration.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        robot_state = scene_state.get_robot_state()
        if self.planned_trajectory is not None:
            robot_state.configuration = self.planned_trajectory.points[-1]
        elif self.fixed_trajectory is not None:
            robot_state.configuration = self.fixed_trajectory.points[-1]
        elif self.fixed_target_configuration is not None:
            robot_state.configuration = self.fixed_target_configuration
        robot_state.frame = self.target_robot_flange_frame
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
    """Action class for linear robotic movements.
    Linear robotic movements are planned by Linear Motion Planners.

    Attributes
    ----------
    polyline_target : list(:class:`compas.geometry.Point`)
        List of points to define a linear movement that is a polyline.
        Specified in world coordinate frame.
        The first point (the starting point) is not required.
        The second point onwards, including the last point (the ending point) is required.

    see :class:`compas_fab.planning.RoboticMovement` for other attributes.
    """

    def __init__(self):
        super(LinearMovement, self).__init__()
        self.polyline_target = []  # type: list(Point)
        self.tag = "Linear Movement"

    @property
    def data(self):
        data = super(LinearMovement, self).data
        data["polyline_target"] = self.polyline_target
        return data

    @data.setter
    def data(self, data):
        super(LinearMovement, type(self)).data.fset(self, data)
        self.polyline_target = data.get("polyline_target", self.polyline_target)


class FreeMovement(RoboticMovement):
    """Action class for free robotic movements.
    Free robotic movements are planned by Free Motion Planners.

    Attributes
    ----------
    intermediate_planning_waypoint : list(:class:`compas.robots.Configuration`)
        List of configurations to define a multi-step free movement.
        The first and last configuration (the starting and target configuration) is not required.
        Only include the intermediate waypoints.
        Waypoints can be used to help the planner 'manually' to find a feasible path around obstacles.
    smoothing_required : bool
        If True, the trajectory smoothing algorithm is invoked after planning to smooth the trajectory.
        Note that smoothing may not be available for all planners. (default: True)
    smoothing_keep_waypoints : bool
        If True, the trajectory smoothing algorithm is allowed to remove the provided waypoints. (default: False)


    see :class:`compas_fab.planning.RoboticMovement` for other attributes.
    """

    def __init__(self):
        super(FreeMovement, self).__init__()
        self.intermediate_planning_waypoint = []  # type: list(Configuration)
        self.smoothing_required = True
        self.smoothing_keep_waypoints = False
        self.tag = "Free Movement"

    @property
    def data(self):
        data = super(FreeMovement, self).data
        data["intermediate_planning_waypoint"] = self.intermediate_planning_waypoint
        data["smoothing_required"] = self.smoothing_required
        data["smoothing_keep_waypoints"] = self.smoothing_keep_waypoints
        return data

    @data.setter
    def data(self, data):
        super(FreeMovement, type(self)).data.fset(self, data)
        self.intermediate_planning_waypoint = data.get("intermediate_planning_waypoint", self.intermediate_planning_waypoint)
        self.smoothing_required = data.get("smoothing_required", self.smoothing_required)
        self.smoothing_keep_waypoints = data.get("smoothing_keep_waypoints", self.smoothing_keep_waypoints)

class OpenGripper(Action):
    """Action to open the gripper.

    Current implementation only changes the attachment state of the workpiece.
    It does not change the configuration of the gripper, even if it is a
    :class:`compas_fab.robots.ToolModel` with kinematic joints.

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
    """Action to load a workpiece, assumed to be performed by a human operator.
    This moves the workpiece to a specific frame.
    Typically used for loading a workpiece into a gripper.
    Can also be used to model the manual attachment of scaffolding (modeled as a :class:`Workpiece`)

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

