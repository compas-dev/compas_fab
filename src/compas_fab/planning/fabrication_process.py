from compas.data import Data

try:
    from compas_fab.robots import Robot  # noqa: F401
    from compas_fab.robots import Tool  # noqa: F401
    from compas_fab.robots import WorkpieceModel  # noqa: F401
    from compas_fab.robots import CollisionMesh  # noqa: F401
except ImportError:
    pass

try:
    from compas_fab.planning import SceneState  # noqa: F401
    from compas_fab.planning import RoboticAction  # noqa: F401
    from compas_fab.planning import Action  # noqa: F401
    from compas_fab.planning import LinearMotion  # noqa: F401
    from compas_fab.planning import FreeMotion  # noqa: F401
except ImportError:
    pass

try:
    from typing import List  # noqa: F401
    from typing import Optional  # noqa: F401
    from typing import Union  # noqa: F401
    from typing import Dict  # noqa: F401
    from typing import Tuple  # noqa: F401
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
