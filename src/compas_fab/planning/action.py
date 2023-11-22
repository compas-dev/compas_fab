from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Point  # noqa: F401
from compas.geometry import Transformation

from compas_fab.robots import Configuration, JointTrajectory  # noqa: F401

try:
    from compas_fab.planning import SceneState  # noqa: F401
except ImportError:
    from .state import SceneState  # noqa: F401

try:
    from typing import Optional  # noqa: F401
    from typing import Tuple  # noqa: F401
except ImportError:
    pass

__all__ = [
    "Action",
    "RoboticAction",
    "LinearMotion",
    "FreeMotion",
    "OpenGripper",
    "CloseGripper",
    "ManuallyMoveWorkpiece",
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
    The only exception are robotic configurations that are changed by :class:`compas_fab.planning.RoboticAction` actions.
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

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        This function can be used to check if the action can be applied to a scene state.
        It can be used to ensure consistancy of the scene state before applying an action.

        Default implementation always returns True. Child classes can override this function
        to implement their own preconditions. This function must not modify the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        This method is called by the assembly process to apply the action to the scene state.
        The SceneState object is modified in place.
        """
        raise NotImplementedError("Action.apply_effects() is not implemented by %s." % type(self))


class RoboticAction(Action):
    """Base class for all robotic movements.

    Robotic movements are actions that changes the robot configuration and
    hence also the frame of the robot flange and all attached tools and workpieces.

    The RoboticAction class only describes the target (ending state)
    of the robotic movement, whereas the starting state is defined using a
    :class:`compas_fab.planning.SceneState` object. The starting state also
    defines the attached tools and workpieces.
    Both objects are required by the motion planner to plan a trajectory.

    After motion planning, the trajectory can be stored in the same RoboticAction class
    and can be used for visualization and execution.

    When applied to a scene state, Robotic movements also changes the state of the
    attached tool and workpiece. If the trajectory have been planned, the configuration
    of the robot is updated to the last configuration of the trajectory. See
    :meth:`compas_fab.planning.RoboticAction.apply_effects` for more details.

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
        super(RoboticAction, self).__init__()
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
        data = super(RoboticAction, self).data
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
        super(RoboticAction, type(self)).data.fset(self, data)
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

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.
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
            attached_workpiece.frame = Frame.from_transformation(
                Transformation.from_frame(robot_state.frame) * attached_workpiece.attached_to_robot_grasp
            )
            if debug:
                print("- Attached Workpiece %s Followed." % attached_workpiece_id)


class LinearMotion(RoboticAction):
    """Action class to describe a Linear robotic movement.

    A linear robotic movement moves the robot flange linearly in Cartesian space. The motion can
    contain multiple linear segments (see intermediate_targets attribute).

    Typically, the orientation between the starting frame and the target frame is the same.
    However some linear motion planners can also interpolate between different orientations, check
    the documentation of the planner for more details.


    Attributes
    ----------
    intermediate_targets : list(:class:`compas.geometry.Frame`), optional
        List of frames to define a linear movement that has multiple linear segments.
        The frames are specified in the world coordinate frame.
        Note that only the intermediate targets are specified, the starting and ending frames
        should not be included in this list. The ending frame is specified by the target_robot_flange_frame attribute.

        Default is an empty list, meaning the movement only has a single linear segment.

    see :class:`compas_fab.planning.RoboticAction` for other attributes.
    """

    def __init__(self):
        super(LinearMotion, self).__init__()
        self.intermediate_targets = []  # type: Optional[list[Point]]
        self.tag = "Linear Movement"

    @property
    def data(self):
        data = super(LinearMotion, self).data
        data["intermediate_targets"] = self.intermediate_targets
        return data

    @data.setter
    def data(self, data):
        super(LinearMotion, type(self)).data.fset(self, data)
        self.intermediate_targets = data.get("intermediate_targets", self.intermediate_targets)


class FreeMotion(RoboticAction):
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


    see :class:`compas_fab.planning.RoboticAction` for other attributes.
    """

    def __init__(self):
        super(FreeMotion, self).__init__()
        self.intermediate_planning_waypoint = []  # type: list(Configuration)
        self.smoothing_required = True
        self.smoothing_keep_waypoints = False
        self.tag = "Free Movement"

    @property
    def data(self):
        data = super(FreeMotion, self).data
        data["intermediate_planning_waypoint"] = self.intermediate_planning_waypoint
        data["smoothing_required"] = self.smoothing_required
        data["smoothing_keep_waypoints"] = self.smoothing_keep_waypoints
        return data

    @data.setter
    def data(self, data):
        super(FreeMotion, type(self)).data.fset(self, data)
        self.intermediate_planning_waypoint = data.get(
            "intermediate_planning_waypoint", self.intermediate_planning_waypoint
        )
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

    def __init__(self, tool_id=None):
        super(OpenGripper, self).__init__()
        self.tool_id = tool_id  # type: Frame

    @property
    def data(self):
        data = super(OpenGripper, self).data
        data["tool_id"] = self.tool_id
        return data

    @data.setter
    def data(self, data):
        super(OpenGripper, type(self)).data.fset(self, data)
        self.tool_id = data.get("tool_id", self.tool_id)

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        This function does not change the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        attached_tool_id = scene_state.get_attached_tool_id()
        if attached_tool_id is None:
            return (False, "Inconsistency: No tool attached to robot.")
        if self.tool_id != attached_tool_id:
            return (False, "Inconsistency: OpenGripper.tool_id does not match the attached tool id.")
        # attached_tool_state = scene_state.get_tool_state(attached_tool_id)
        # TODO: Check if tool is a gripper
        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.
        It is possible to open the gripper with or without a workpiece attached.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """

        # TODO: Change the configuration of the gripper to opened

        # Transform attached workpieces
        attached_workpiece_id = scene_state.get_attached_workpiece_id()
        if debug:
            print("Gripper opened.")
        if attached_workpiece_id is not None:
            attached_workpiece = scene_state.get_workpiece_state(attached_workpiece_id)
            attached_workpiece.attached_to_robot = False
            attached_workpiece.attached_to_robot_grasp = None
            if debug:
                print("- Workpiece %s detached from tool." % attached_workpiece_id)


class CloseGripper(Action):
    """Action to close the gripper.

    If the gripper is closed around a workpiece, the workpiece is attached to the gripper.

    Attributes
    ----------
    tool_id : str
        The id of the gripper tool that is used.
    attaching_workpiece_id : str, optional
        The id of the workpiece attached to the gripper.
        If no workpiece is attached during the gripper closure, None.
    attaching_workpiece_grasp : :class:`compas.geometry.Transformation`, optional
        If attaching_workpiece_id is not None, the grasp frame of the workpiece
        relative to the robot flange. Default is the identity transformation.
        If attaching_workpiece_id is None, this attribute is meaningless.
    """

    def __init__(self, tool_id=None, attaching_workpiece_id=None, attaching_workpiece_grasp=Transformation()):
        super(CloseGripper, self).__init__()
        self.tool_id = tool_id  # type: Frame
        self.attaching_workpiece_id = attaching_workpiece_id  # type: Optional[str]
        self.attaching_workpiece_grasp = attaching_workpiece_grasp  # type: Optional[Transformation]

    @property
    def data(self):
        data = super(CloseGripper, self).data
        data["attaching_workpiece_id"] = self.attaching_workpiece_id
        data["attaching_workpiece_grasp"] = self.attaching_workpiece_grasp
        return data

    @data.setter
    def data(self, data):
        super(CloseGripper, type(self)).data.fset(self, data)
        self.attaching_workpiece_id = data.get("attaching_workpiece_id", self.attaching_workpiece_id)
        self.attaching_workpiece_grasp = data.get("attaching_workpiece_grasp", self.attaching_workpiece_grasp)

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        This function does not change the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        attached_tool_id = scene_state.get_attached_tool_id()
        if attached_tool_id is None:
            return (False, "Inconsistency: No tool attached to robot.")
        if self.tool_id != attached_tool_id:
            return (False, "Inconsistency: CloseGripper.tool_id does not match the attached tool id.")
        if self.attaching_workpiece_id is not None:
            attached_workpiece_id = scene_state.get_attached_workpiece_id()
            if attached_workpiece_id == self.attaching_workpiece_id:
                return (False, "Inconsistency: Workpiece is already attached to the tool.")
            if attached_workpiece_id is not None:
                return (False, "Inconsistency: Another workpiece is already attached to the tool.")
            if self.attaching_workpiece_id not in scene_state.workpiece_states:
                return (False, "Inconsistency: Workpiece is not in the scene.")
        # TODO: Check if tool is a gripper
        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.
        It is possible to open the gripper with or without a workpiece attached.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        # TODO: Change the configuration of the gripper to closed

        if debug:
            print("Gripper closed.")
        # Transform attached workpieces
        if self.attaching_workpiece_id is not None:
            workpiece_state = scene_state.get_workpiece_state(self.attaching_workpiece_id)
            # Update the workpiece grasp and frame
            workpiece_state.attached_to_robot = True
            workpiece_state.attached_to_robot_grasp = self.attaching_workpiece_grasp or Transformation()
            robot_flange_frame = scene_state.get_robot_state().frame
            workpiece_state.frame = Frame.from_transformation(
                Transformation.from_frame(robot_flange_frame) * workpiece_state.attached_to_robot_grasp
            )
            if debug:
                print("- Workpiece %s attached to tool." % self.attaching_workpiece_id)


class ManuallyMoveWorkpiece(Action):
    """Operator action to move a workpiece from one place to another.
    This moves the workpiece to a specific frame.
    Typically used for loading a workpiece into a gripper.
    Can also be used to model the manual attachment of scaffolding (modeled as a :class:`Workpiece`)

    Attributes
    ----------
    workpiece_id : str
        The id of the workpiece to be moved.
    frame : :class:`compas.geometry.Frame`
        The frame of the workpiece at the target location.
        Specified in world coordinate frame.
    """

    def __init__(self, workpiece_id=None, frame=Frame.worldXY()):
        super(ManuallyMoveWorkpiece, self).__init__()
        self.workpiece_id = workpiece_id  # type: str
        self.frame = frame  # type: Frame

    @property
    def data(self):
        data = super(ManuallyMoveWorkpiece, self).data
        data["workpiece_id"] = self.workpiece_id
        data["frame"] = self.frame
        return data

    @data.setter
    def data(self, data):
        super(ManuallyMoveWorkpiece, type(self)).data.fset(self, data)
        self.workpiece_id = data.get("workpiece_id", self.workpiece_id)
        self.frame = data.get("frame", self.frame)

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        This function does not change the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        if self.workpiece_id not in scene_state.workpiece_states:
            return (False, "Inconsistency: Workpiece is not in the scene.")
        workpiece_state = scene_state.get_workpiece_state(self.workpiece_id)
        if workpiece_state.attached_to_robot is True:
            return (False, "Inconsistency: Workpiece is already attached to the robot.")
        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        # Transform attached objects
        workpiece_state = scene_state.get_workpiece_state(self.workpiece_id)
        workpiece_state.frame = self.frame
        if debug:
            print("Workpiece %s loaded to new location." % self.workpiece_id)


class HideWorkpieces(Action):
    """Action to hide workpieces. Can be used to model objects that are removed the scene.

    This changes the WorkpieceState.is_hidden property.
    Can also be used to model the manual detachment of scaffolding (modeled as a :class:`Workpiece`)

    Attributes
    ----------
    workpiece_ids : list(str)
        List of workpiece ids to be hidden.
    """

    def __init__(self, workpiece_ids=[]):
        super(HideWorkpieces, self).__init__()
        self.workpiece_ids = workpiece_ids  # type: list[str]

    @property
    def data(self):
        data = super(HideWorkpieces, self).data
        data["workpiece_ids"] = self.workpiece_ids
        return data

    @data.setter
    def data(self, data):
        super(HideWorkpieces, type(self)).data.fset(self, data)
        self.workpiece_ids = data.get("workpiece_ids", self.workpiece_ids)

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        All the workpiece_ids must be:
        - in the scene
        - not hidden
        - not attached to the robot

        This function does not change the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        for workpiece_id in self.workpiece_ids:
            if workpiece_id not in scene_state.workpiece_states:
                return (False, "Inconsistency: Workpiece %s is not in the scene." % workpiece_id)
            workpiece_state = scene_state.get_workpiece_state(workpiece_id)
            if workpiece_state.is_hidden:
                return (False, "Inconsistency: Workpiece %s is already hidden." % workpiece_id)
            if workpiece_state.attached_to_robot:
                return (False, "Inconsistency: Workpiece %s is attached to the robot." % workpiece_id)

        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        for workpiece_id in self.workpiece_ids:
            workpiece_state = scene_state.get_workpiece_state(workpiece_id)
            workpiece_state.is_hidden = True
            if debug:
                print("Workpiece %s is hidden." % workpiece_id)


class ShowWorkpieces(Action):
    """Action to show workpieces. Can be used to model objects that are added to the scene.

    This changes the WorkpieceState.is_hidden property.
    Can also be used to model the manual attachment of scaffolding (modeled as a :class:`Workpiece`)

    Attributes
    ----------
    workpiece_ids : list(str)
        List of workpiece ids to be shown.
    """

    def __init__(self, workpiece_ids=[]):
        super(ShowWorkpieces, self).__init__()
        self.workpiece_ids = workpiece_ids  # type: list[str]

    @property
    def data(self):
        data = super(ShowWorkpieces, self).data
        data["workpiece_ids"] = self.workpiece_ids
        return data

    @data.setter
    def data(self, data):
        super(ShowWorkpieces, type(self)).data.fset(self, data)
        self.workpiece_ids = data.get("workpiece_ids", self.workpiece_ids)

    def check_preconditions(self, scene_state):
        # type: (SceneState) -> Tuple(bool, str)
        """Checks if the action can be applied to a scene state.

        All the workpiece_ids must be:
        - in the scene
        - hidden
        - not attached to the robot

        This function does not change the scene state.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.

        Returns
        -------
        Tuple(bool, str)
            A tuple of a boolean and a string.
            The boolean indicates if the action can be applied to the scene state.
            The string is a human readable message that describes the reason for failure.
        """
        for workpiece_id in self.workpiece_ids:
            if workpiece_id not in scene_state.workpiece_states:
                return (False, "Inconsistency: Workpiece %s is not in the scene." % workpiece_id)
            workpiece_state = scene_state.get_workpiece_state(workpiece_id)
            if not workpiece_state.is_hidden:
                return (False, "Inconsistency: Workpiece %s is not hidden." % workpiece_id)
            if workpiece_state.attached_to_robot:
                return (False, "Inconsistency: Workpiece %s is attached to the robot." % workpiece_id)

        return (True, None)

    def apply_effects(self, scene_state, debug=False):
        # type: (SceneState, bool) -> None
        """Applies the action to a scene state.

        The SceneState is modified in place with the effect of the Action.

        Parameters
        ----------
        scene_state : :class:`compas_fab.planning.SceneState`
            The scene state to apply the action to.
        """
        for workpiece_id in self.workpiece_ids:
            workpiece_state = scene_state.get_workpiece_state(workpiece_id)
            workpiece_state.is_hidden = False
            if debug:
                print("Workpiece %s is shown." % workpiece_id)
