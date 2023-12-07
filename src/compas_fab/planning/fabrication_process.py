from compas.data import Data

try:
    from compas_fab.robots import CollisionMesh  # noqa: F401
    from compas_fab.robots import Robot  # noqa: F401
    from compas_fab.robots import Tool  # noqa: F401
    from compas_fab.robots import WorkpieceModel  # noqa: F401
except ImportError:
    pass

try:
    from compas_fab.planning import Action  # noqa: F401
    from compas_fab.planning import FreeMotion  # noqa: F401
    from compas_fab.planning import LinearMotion  # noqa: F401
    from compas_fab.planning import RoboticAction  # noqa: F401
    from compas_fab.planning import SceneState  # noqa: F401
except ImportError:
    pass

try:
    from typing import Dict  # noqa: F401
    from typing import List  # noqa: F401
    from typing import Optional  # noqa: F401
    from typing import Tuple  # noqa: F401
    from typing import Type  # noqa: F401
    from typing import Union  # noqa: F401
except ImportError:
    pass


class FabricationProcess(Data):
    """Represents a robotic fabrication process.

    The FabricationProcess class is the main base class for modeling a robotic fabrication process.
    It contains the reference to all the tangible objects in the process: the robot,
    the tool(s), the workpiece(s) and the static object(s). The current FabricationProcess implementation
    supports only a single robot but supports multiple tools, workpieces and static objects. All the objects
    are assumed to be present in the scene from the beginning of the process until the end. Temporary objects that
    are added or removed during the process must be modelled by manpulating the `is_hidden` property of the objects.
    This assumption and limitation cannot be changed by inhereting from this class.

    All tool(s), workpiece(s) and static object(s) require a unique identifier name.
    The names are used when specifing details of the Actions, such as the allowable collisions pairs
    during planning (see :ref:`Allowable Collision Matrix <acm>`).
    It maybe useful to give the object names a prefix (e.g. T1, T2 ... for tools, W1, W2 ... for workpieces)
    to make debug messages and logs easier to read.

    The FabricationProcess class contains functions to support Task Planning (the order of actions) and
    Motion Planning (the trajectory of robotic motions). Actions are assumed to be executed sequentially
    by the robot and hence stored in a List. Concurrent actions are not supported by default but can be
    implemented by creating custom Action classes, See :ref:`custom_actions` for more details.

    The FabricationProcess class is intended to be the top-level class that is serialized and deserialized
    between different stages of the process. Typically the FabricationProcess object is created during the
    design stage and can be used to simulate and visualize the process in a virtual environment. The design stage
    is followed by the planning stage, where the RoboticMotion in the FabricationProcess are motion planed.
    The resulting FabricationProcess object can be used to execute the process on a real robot.

    At the moment, the FabricationProcess class is intended to be used as a data container for the process and
    does not contain any version control or change tracking features. It is left to the user to implement
    change tracking if back-and-forth is needed between the design and planning stages.
    One option is to use GIT to track changes in the serialized FabricationProcess object.

    The State of all the tangible objects in the scene at any given time is represented by the SceneState class.
    Only the initial state can be set manually, the intermediate states after each action are computed automatically
    by the FabricationProcess class based on the initial state and the list of actions.
    See :ref:`Action and State <action_state>` for more details.

    The FabricationProcess class is supported by the FabricationProcessArtist, which can be used to visualize
    the process in a virtual environment. It supports visualization of the process throughout the design to production
    workflow, even before all the planning is completed. After Task Planning (where list of actions are created) the
    states before and after each action can be visualized. After Motion Planning, the RobotModel can be animated to simulate
    RobotMotions. See :ref:`fabrication_process_visualization` for more details.

    After successful planning, the FabricationProcess class contains all the actions and motion trajectory for executing the
    process on a real robot. A user-implmented code is needed for transcribing the actions and motion trajectory into
    robot-specific commands. It will also need to maintain communication with the robot drivers, monitor execution progress and
    provide an operator interface for monitoring and control (e.g. pause, resume, stop).
    See :ref:`FabricationProcess Execution <fabrication_process_execution>` for more details.

    Todo
    ----
    Move this long description to a tutorial page.

    Note
    ----
    The FabricationProcess class can be inhereted to create a specific process, such as a PickAndPlaceProcess
    or ExtrusionProcess. The inhereted class can keep track of process specific objects, for example, temporary
    scaffolding (for an assembly process) or the support structure (for an extrusion process). The inhereted
    class can also implement process specific Task Planning and Motion Planning functions, for example, to
    add gripper opening and closing actions (for an assembly process) or to add extruder actions (for an
    extrusion process). See :ref:`Custom Fabrication Process <custom_fabrication_process>` for more details.

    The FabricationProcess class aims to be a generic base class and as such does not contain any function for Task Planning.
    This is left to the inhereted class. However, once the list of actions are created,
    it contains generic function to validate the sequence of actions and to plan their trajectory (Motion Planning).
    The Motion Planning functions assumes that the actions are sequentially executed by the robot, and that
    the resulting state of the first action is used as the initial state for the next action, and so on.
    See :ref:`planning_process` for more details.


    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`
        The robot that is used for planning and executing the process.
        The Robot object contains the :class:`compas.robots.RobotModel` and the :class:`compas_fab.robots.RobotSemantics`.
        Note that Robot.client and Robot.artist are not serialized and must be attached manually after deserialization.

    robot_planning_group : str, optional
        The name of the robot planning group that is used for planning and executing the process.
        This is used to differentiate between different robot flanges for robots with multiple arms / flanges.
        The value corrisponds to the name of the planning groups defined in RobotSemantics (originates from the SRDF).
        The default implementation of the FabricationProcess class supports only a single robot planning group.
        If not specified (default), the main_group_name in RobotSemantics is used.

    tools : dict of :class:`compas_fab.robots.Tool`
        A dictionary of tools that are used in the process.
        The key must equal to the value of `Tool.name` and must be unique for each tool in the process.
        If multiple tools are used in the process, as with a tool changer, each tool must have a unique name.
        Typically the name has a prefix "T" (e.g. T1, T2 ...).

    workpieces : dict of :class:`WorkpieceModel`
        A dictionary of workpieces that are used in the process.
        The key must equal to the value of `WorkpieceModel.name` and must be unique for each workpiece in the process.
        Typically the name of the workpiece is the same as the name of the object in the CAD model, or design model,
        but this is not required.
        Typically the name has a prefix "W" (e.g. W1, W2 ...).

    static_objects : dict of :class:`compas_fab.robots.CollisionMesh`
        A dictionary of static objects that are used in the process.
        The key must equal to the value of `CollisionMesh.id` and must be unique for each static object in the process.
        Typically the name has a prefix "S" (e.g. S1, S2 ...).

    initial_scene_state : :class:`SceneState`
        The initial state of the scene before the process starts.
        The initial state is used to plan the first action in the process, it can be defined manually or automatically based
        on process-specific logic. For example, PickAndPlaceProcess and ExtrusionProcess have helper functions to define
        the initial state.
        See :ref:`Initial Scene State <initial_scene_state>` for more details.

    actions : list of :class:`Action`
        A list of actions that are executed sequentially by the robot.
        The list of actions are typically created using a helper function that is specific to the process.
        See :ref:`Task Planning <task_planning>` for more details.
        The list of actions contains RoboticMotion objects that requires Motion Planning for finding their trajectory.
        See ::ref:`Chained Motion Planning <chained_motion_planning>` for more details.
    """

    def __init__(self):
        self.robot = None  # type: Robot
        self.robot_planning_group = None  # type: Optional[str]
        self.tools = {}  # type: Dict[str, Tool]
        self.workpieces = {}  # type: Dict[str, WorkpieceModel]
        self.static_objects = {}  # type: Dict[str, CollisionMesh]
        self.initial_scene_state = None  # type: SceneState
        self.actions = []  # type: List[Action]

    def __str__(self):
        robot_name = self.robot.name if self.robot else "None"
        return "FabricationProcess(robot={}, {} tools, {} workpieces, {} static_objects, {} actions)".format(
            robot_name, len(self.tools), len(self.workpieces), len(self.static_objects), len(self.actions)
        )

    @property
    def data(self):
        data = {
            "robot": self.robot,
            "robot_planning_group": self.robot_planning_group,
            "tools": self.tools,
            "workpieces": self.workpieces,
            "static_objects": self.static_objects,
            "initial_scene_state": self.initial_scene_state,
            "actions": self.actions,
        }
        return data

    @data.setter
    def data(self, data):
        self.robot = data["robot"]
        self.robot_planning_group = data["robot_planning_group"]
        self.tools = data["tools"]
        self.workpieces = data["workpieces"]
        self.static_objects = data["static_objects"]
        self.initial_scene_state = data["initial_scene_state"]
        self.actions = data["actions"]

    # ##################
    # Validation methods
    # ##################

    def validate_task_planning_ready(self):
        # type: () -> Tuple[bool, Optional[str]]
        """Validates that the process is ready for Task Planning.

        This function is intended to be called after the initial_scene_state before the list of actions are created.
        It validates that the initial_scene_state is set and that the list of actions are valid.
        See :ref:`Task Planning <task_planning>` for more details.

        If the fabrication process is valid, this function returns None without raising any exceptions.

        Returns
        ------
        (True, None)
        - If the fabrication process is valid.

        (False, str)
        - If the initial_scene_state is not set.
        - If the robot is not set or if the robot_planning_group is not set or if the robot_planning_group is not present in the robot.
        - If there are no tools in the process.
        - If the names of the tools, workpieces and static objects are not unique.
        - If the names of the tools and workpieces are not present in the initial_scene_state.

        """
        # Check that the initial_scene_state is set
        if not self.initial_scene_state:
            raise Exception("Initial scene state is not set.")

        # Check that the robot and robot_planning_group are set
        if not self.robot:
            raise Exception("Robot is not set.")
        if not self.robot_planning_group:
            raise Exception("Robot planning group is not set.")
        if self.robot_planning_group not in self.robot.semantics.groups:
            raise Exception("Robot planning group is not present in the robot.")

        # Check that there are tools in the process
        if not self.tools:
            raise Exception("There are no tools in the process.")

        # Check that the names of the tools, workpieces and static objects are unique
        names = []
        names.extend(self.tools.keys())
        names.extend(self.workpieces.keys())
        names.extend(self.static_objects.keys())
        if len(names) != len(set(names)):
            raise KeyError("The names of the tools, workpieces and static objects are not unique.")

        # Check that the names of the tools, workpieces and static objects are present in the initial_scene_state
        for key in self.tools.keys():
            if key not in self.initial_scene_state.tool_states.keys():
                raise KeyError("The name of the tool '{}' is not present in the initial_scene_state.".format(key))
        for key in self.workpieces.keys():
            if key not in self.initial_scene_state.workpiece_states.keys():
                raise KeyError("The name of the workpiece '{}' is not present in the initial_scene_state.".format(key))

    def validate_motion_planning_ready(self):
        # type: () -> Tuple[bool, str]
        """Validates that the process is ready for Motion Planning.

        This function is intended to be called after the list of actions are created.
        It validates that the list of actions are valid.
        See :ref:`Motion Planning <motion_planning>` for more details.

        If the fabrication process is valid, this function returns None without raising any exceptions.

        Returns
        ------
        (True, None)
        - If the fabrication process is valid.

        (False, str)
        - If the list of actions are empty.
        - If the trajectory of RoboticMotions are not planned.
        - If the trajectory of RoboticMotions are not valid.

        """
        # Check that the list of actions are not empty
        if not self.actions:
            return (False, "The list of actions are empty.")

        # Check that the trajectory of RoboticMotions are planned
        for action in self.actions:
            if isinstance(action, RoboticAction):
                if not action.trajectory:
                    raise (False, "The trajectory of RoboticMotions are not planned.")

        # Check that the trajectory of RoboticMotions are valid
        for action in self.actions:
            if isinstance(action, RoboticAction):
                if not action.trajectory.is_valid():
                    raise (False, "The trajectory of RoboticMotions are not valid.")

        return (True, None)

    # #######################
    # Actions related methods
    # #######################

    def get_robotic_actions(self):
        # type: () -> list[RoboticAction]
        """Filter the action list and returns only the RoboticAction(s).

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created.
        """
        return [action for action in self.actions if isinstance(action, RoboticAction)]

    def get_next_action_of_type(self, action, type_filter=None):
        # type: (Action, Optional[Type]) -> (Action|None)
        """Returns the next robotic action in the list of actions (FabricationProcess.actions).

        If type_filter is specified, returns the next action that is an instance of the specified type.
        For example, if type is :class:`RoboticAction`, the returned action can be any subclass of RoboticAction,
        such as :class:`FreeMotion` or class:`LinearMotion`.

        If there are no more actions in the list, returns None.

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created.

        Parameters
        ----------
        action : :class:`compas_fab.planning.Action`, optional
            The current action to query.
            The action must be present in the list of actions (FabricationProcess.actions).
            If None, the first robotic action in the list of actions is returned.
        type_filter : Type, optional , default None
            The type of the next robotic action to query. The type must be a subclass of
            :class:`compas_fab.planning.Action`

        Exceptions
        ----------
        ValueError
            If the action is not present in the list of actions (FabricationProcess.actions).
        TypeError
            If the type is not a subclass of :class:`compas_fab.planning.Action`.
        """
        # Raise error if type is not a subclass of Action
        if action is not None and not issubclass(type_filter, Action):
            raise TypeError("type must be a subclass of Action.")

        # Use the index of the action to find the next action
        if action is None:
            action_index = 0  # Start searching from the first action
        else:
            action_index = self.actions.index(action) + 1  # Start searching from the next action

        # Search for the next action that is an instance of the specified type
        while action_index < len(self.actions):
            next_action = self.actions[action_index]
            if type_filter is None or isinstance(next_action, type_filter):
                return next_action
            action_index += 1  # Continue searching next action

        # Search failed, return None
        return None

    def get_previous_action_of_type(self, action, type_filter=None):
        # type: (Action, Optional[Type]) -> (Action|None)
        """Returns the previous robotic action in the list of actions (FabricationProcess.actions).

        If type_filter is specified, returns the previous action that is an instance of the specified type.

        See Also
        --------
        :meth:`get_next_action_of_type`

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created.

        Parameters
        ----------
        action : :class:`compas_fab.planning.Action`, optional
            The current action to query.
            The action must be present in the list of actions (FabricationProcess.actions).
            If None, the last robotic action in the list of actions is returned.
        type_filter : Type, optional , default None
            The type of the previous robotic action to query. The type must be a subclass of
            :class:`compas_fab.planning.Action`

        Exceptions
        ----------
        ValueError
            If the action is not present in the list of actions (FabricationProcess.actions).
        TypeError
            If the type is not a subclass of :class:`compas_fab.planning.Action`.
        """
        # Raise error if type is not a subclass of Action
        if action is not None and not issubclass(type_filter, Action):
            raise TypeError("type must be a subclass of Action.")

        # Use the index of the action to find the previous action
        if action is None:
            action_index = len(self.actions) - 1  # Start searching from the last action
        else:
            action_index = self.actions.index(action) - 1  # Start searching from the previous action

        # Search for the previous action that is an instance of the specified type
        while action_index >= 0:
            previous_action = self.actions[action_index]
            if type_filter is None or isinstance(previous_action, type_filter):
                return previous_action
            action_index -= 1  # Continue searching previous action

        # Search failed, return None
        return None

    def get_next_robotic_action(self, action):
        # type: (RoboticAction) -> Optional[RoboticAction]
        """Returns the next robotic action in the list of actions (FabricationProcess.actions).

        If there are no more robotic actions in the list, returns None.

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created.

        See Also
        --------
        :meth:`get_next_action_of_type`

        Parameters
        ----------
        action : :class:`compas_fab.planning.RoboticAction`, optional
            The current ction to query.
            The action must be present in the list of actions (FabricationProcess.actions).
            If None, the first robotic action in the list of actions is returned.
        """
        return self.get_next_action_of_type(action, RoboticAction)

    def get_previous_robotic_action(self, action):
        # type: (Optional[RoboticAction]) -> Optional[RoboticAction]
        """Returns the previous robotic action in the list of actions (FabricationProcess.actions).

        If there are no more robotic actions in the list, returns None.

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created.

        See Also
        --------
        :meth:`get_previous_action_of_type`

        Parameters
        ----------
        action : :class:`compas_fab.planning.RoboticAction`, optional
            The current action to query.
            The action must be present in the list of actions (FabricationProcess.actions).
            If None, the last robotic action in the list of actions is returned.
        """
        return self.get_previous_action_of_type(action, RoboticAction)

    # #####################
    # State related methods
    # #####################

    def get_intermediate_scene_state(self, state_index, debug=False):
        # type: (int, bool) -> SceneState
        """Parses the actions in the process and returns an intermediate scene state of the process.

        Starting from the initial state, this function iteratively applies the actions in the process
        using :meth:`Action.apply_effects` until the specified state_index is reached.
        The returned scene state is after applying action[state_index].

        Note
        ----
        This function can only be used after the list of actions (FabricationProcess.actions) are created
        and the :attr:`initial_scene_state` is set.

        Parameters
        ----------
        state_index : int
            The index of state to return. Index 0 is equal to the initial state.
            The index must be smaller than or equal to the number of actions in the process.
        debug : bool, optional, default False
            If True, prints debug messages when iteratively applying the actions.

        Execption
        ---------
        IndexError
            If the state_index is larger than the number of actions in the process.
            If the state_index is negative.
        ValueError
            If the initial_scene_state is not set.
        """
        if state_index < 0:
            raise IndexError("state_index must be larger than or equal to 0.")
        if state_index <= len(self.actions):
            raise IndexError("state_index must be smaller than or equal to the number of actions in the process.")
        if not self.initial_scene_state:
            raise ValueError("Initial scene state is not set.")

        # A copy of the initial state is used to avoid modifying the initial state
        scene_state = self.initial_scene_state.copy()
        for action in self.actions[:state_index]:
            action.apply_effects(scene_state, debug=debug)
        return scene_state
