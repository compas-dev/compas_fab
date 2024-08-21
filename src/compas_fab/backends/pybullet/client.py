from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools
import os
import sys
import tempfile

import compas
from compas.colors import Color

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas_robots import RobotModel
from compas_robots.files import URDF
from compas_robots.model import MeshDescriptor
from compas_robots import ToolModel
from compas_robots.model import Joint
from compas_robots.model import Material

from compas_fab.backends import CollisionCheckError
from compas_fab.backends.interfaces.client import ClientInterface

from compas_fab.robots import Robot
from compas_fab.robots import RobotLibrary
from compas_fab.robots import RobotSemantics
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.utilities import LazyLoader

from . import const
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .utils import LOG
from .utils import redirect_stdout


if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401
        from typing import Optional  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas_robots.resources import AbstractMeshLoader  # noqa: F401
        from compas_fab.robots import RigidBody  # noqa: F401
        from compas_robots.model import Mimic

        from compas_fab.backends.kinematics import AnalyticalInverseKinematics  # noqa: F401
        from compas_fab.backends.kinematics import AnalyticalPlanCartesianMotion  # noqa: F401

        # Load pybullet for type hinting
        import pybullet

# If Pybullet is not defined, load it from LazyLoader
if "pybullet" not in sys.modules:
    pybullet = LazyLoader("pybullet", globals(), "pybullet")  # noqa: F811

__all__ = [
    "PyBulletClient",
    "AnalyticalPyBulletClient",
]


class PyBulletBase(object):
    def __init__(self, connection_type):
        # type: (str) -> None
        self.client_id = None
        self.connection_type = connection_type
        super(PyBulletBase, self).__init__()

    # -------------------------------------------------------------------
    # Functions for connecting and disconnecting from the PyBullet server
    # -------------------------------------------------------------------

    def connect(self, shadows=True, color=None, width=None, height=None):
        # type: (bool, Tuple[float,float,float], int, int) -> None
        """Connect from the PyBullet server.

        Parameters
        ----------
        shadows : :obj:`bool`
            Display shadows in the GUI. Defaults to ``True``.
        color : :obj:`tuple` of :obj:`float`
            Set the background color of the GUI. Defaults to ``None``.
        width : :obj:`int`
            Set the width in pixels of the GUI. Defaults to ``None``.
        height : :obj:`int`
            Set the height in pixels of GUI. Defaults to ``None``.

        Returns
        -------
        ``None``
        """
        # Shared Memory: execute the physics simulation and rendering in a separate process
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7

        # If connection_type is 'gui' but there is no display detected, switch to 'direct'
        if self.connection_type == "gui" and not compas.OSX and not compas.WINDOWS and ("DISPLAY" not in os.environ):
            self.connection_type = "direct"
            print("No display detected! Continuing without GUI.")

        # Format GUI parameters for launching PyBullet
        options = self._compose_options(color, width, height)

        with redirect_stdout():
            self.client_id = pybullet.connect(const.CONNECTION_TYPE[self.connection_type], options=options)

        if self.client_id < 0:
            raise Exception("Error in establishing connection with PyBullet.")
        if self.connection_type == "gui":
            self._configure_debug_visualizer(shadows)

    @staticmethod
    def _compose_options(color, width, height):
        options = ""
        if color is not None:
            options += "--background_color_red={} --background_color_green={} --background_color_blue={}".format(*color)
        if width is not None:
            options += "--width={}".format(width)
        if height is not None:
            options += "--height={}".format(height)
        return options

    def _configure_debug_visualizer(self, shadows):
        # COV_ENABLE_GUI = False turns off the sidebar and parameter views in the GUI
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(
            pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=self.client_id
        )
        pybullet.configureDebugVisualizer(
            pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=self.client_id
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, shadows, physicsClientId=self.client_id)

    def disconnect(self):
        """Disconnect from the PyBullet server."""
        with redirect_stdout():
            return pybullet.disconnect(physicsClientId=self.client_id)

    @property
    def is_connected(self):
        """Indicates whether the client has an active connection.

        Returns
        -------
        :obj:`bool`
            ``True`` if connected, ``False`` otherwise.
        """
        if self.client_id is None:
            return False
        return pybullet.getConnectionInfo(physicsClientId=self.client_id)["isConnected"] == 1


class PyBulletClient(PyBulletBase, ClientInterface):
    """Interface to use pybullet as backend.

    :class:`compas_fab.backends.PyBulletClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

    Thanks to Yijiang Huang and his work in `pybullet_planning
    <https://github.com/yijiangh/pybullet_planning>`_ for much inspiration.

    Parameters
    ----------
    connection_type : :obj:`str`
        Sets the connection type.
        ``'gui'`` for a graphical user interface (default).
        ``'direct'`` mode will create a headless physics engine and directly communicates with it.
        ``'shared_memory'``, ``'udp'``, and ``'tcp'`` are not supported.

    verbose : :obj:`bool`
        Use verbose logging. Defaults to ``False``.

    Attributes
    ----------
    verbose : :obj:`bool`
        Use verbose logging.
    rigid_bodies_puids : :obj:`dict` of (:obj:`str`, :obj:`list` of :obj:`int`)
        Dictionary of rigid bodies and their PyBullet ids.

    Examples
    --------
    >>> from compas_fab.backends import PyBulletClient
    >>> with PyBulletClient(connection_type='direct') as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: True

    """

    def __init__(self, connection_type="gui", verbose=False):
        # type (str, bool) -> None
        super(PyBulletClient, self).__init__(connection_type)
        self.verbose = verbose

        # Robot Cell
        # The Pybullet client will not keep track of the Robot object directly but uses the one embedded in the RobotCell
        self._robot_cell = None
        self._robot_cell_state = None

        # PyBullet unique id
        self.robot_puid = None
        # Each robot joint has a unique id
        self.robot_joint_puids = {}  # type: dict[str, int]
        # Each robot link has a unique id
        self.robot_link_puids = {}  # type: dict[str, int]

        # Each RigidBody can have multiple meshes, so we store a list of puids
        self.rigid_bodies_puids = {}  # type: dict[str, List[int]]
        # Each ToolModel is a single URDF, so we store a single puid
        self.tools_puids = {}  # type: dict[str, int]

        self.disabled_collisions = set()
        self._cache_dir = None

    @property
    def robot_cell(self):
        # type: () -> RobotCell
        """The robot cell that is currently loaded in the PyBullet server."""
        return self._robot_cell

    @property
    def robot(self):
        # type: () -> Robot
        """The robot that is currently loaded in the PyBullet server."""
        return self.robot_cell.robot

    @property
    def robot_cell_state(self):
        # type: () -> RobotCellState
        """The state of the robot cell that is currently loaded in the PyBullet server.

        There is typically no reason to access this directly, as the state is managed by the client.
        However, it can be useful for debugging or introspection.
        """
        return self._robot_cell_state

    def __enter__(self):
        self._cache_dir = tempfile.TemporaryDirectory(prefix="compas_fab")
        self.connect()
        return self

    def __exit__(self, *args):
        self._cache_dir.cleanup()
        self.disconnect()

    @property
    def unordered_disabled_collisions(self):
        # type: () -> set
        """Returns the set of disabled collisions between robot links."""
        return {frozenset(pair) for pair in self.disabled_collisions}

    def step_simulation(self):
        """By default, the physics server will not step the simulation,
        unless you explicitly send a ``step_simulation`` command.  This
        method will perform all the actions in a single forward dynamics
        simulation step such as collision detection, constraint solving
        and integration. The timestep is 1/240 second.
        """
        pybullet.stepSimulation(physicsClientId=self.client_id)

    # ------------------------------------------
    # Functions for loading and removing objects
    # ------------------------------------------

    def set_robot(self, robot, concavity=False):
        # type: (Robot, Optional[bool]) -> Robot
        """Load an existing robot to PyBullet.

        This function is used by SetRobotCell and should not be called directly by user.
        The robot must contain geometry and semantics.

        If a robot is already loaded, it will be replaced by the new robot.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot to be saved for use with PyBullet.
        concavity : :obj:`bool`, optional

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            A robot instance.
        """
        if self.robot_puid is not None:
            self.remove_robot()

        urdf_fp = self.robot_model_to_urdf(robot.model, concavity=concavity)
        self.robot_puid = robot_puid = self.pybullet_load_urdf(urdf_fp)

        # The root link of the robot have index of `-1` (refer to PyBullet API)
        self.robot_link_puids[robot.model.root.name] = -1

        # For the rest of the joints and links, their numbers are the same
        # The id of the joint is the same as the id of its child link
        for id in range(self._get_num_joints(robot_puid)):
            joint_name = self._get_joint_name(id, robot_puid)
            self.robot_joint_puids[joint_name] = id
            link_name = self._get_link_name(id, robot_puid)
            self.robot_link_puids[link_name] = id

        # Note: Mimic joints are considered as a DOF in Pybullet and most functions
        # that accept poses will require passing also the mimic joints.
        # In compas_fab, mimic joints are not considered as DOF and are not included in get_configurable_joints().

        self.disabled_collisions = robot.semantics.disabled_collisions

        return robot

    def remove_robot(self):
        """Remove the robot from the PyBullet server if it exists."""
        if self.robot_puid is None:
            return
        pybullet.removeBody(self.robot_puid, physicsClientId=self.client_id)
        self.robot_puid = None
        self.robot_joint_puids = {}
        self.robot_link_puids = {}

    def add_tool(self, name, tool_model):
        # type: (str, ToolModel) -> Robot
        """Load a ToolModel object to PyBullet.

        Parameters
        ----------
        name : :obj:`str`
            The name of the tool.
        tool_model : :class:`compas_robot.ToolModel`
            The tool_model to be loaded into PyBullet.

        """
        file_path = self.robot_model_to_urdf(tool_model)
        pybullet_uid = self.pybullet_load_urdf(file_path)
        self.tools_puids[name] = pybullet_uid

    def remove_tool(self, name):
        # type: (str) -> None
        """Remove a tool from the PyBullet server if it exists.

        Parameters
        ----------
        name : :obj:`str`
            The name of the tool.
        """
        if name not in self.tools_puids:
            return
        pybullet.removeBody(self.tools_puids[name], physicsClientId=self.client_id)
        del self.tools_puids[name]

    def add_rigid_body(self, name, rigid_body, concavity=False, mass=const.STATIC_MASS):
        # type: (str, RigidBody, bool, float) -> int
        tmp_obj_path = os.path.join(self._cache_dir.name, "{}.obj".format(rigid_body.guid))

        # Rigid bodies can have multiple meshes in them, at the moment we join them together as a single mesh
        # If there are problems with this, we can change it to exporting individual obj files per mesh to Pybullet
        mesh = Mesh()
        for m in rigid_body.get_collision_meshes:
            mesh.join(m, precision=12)
        mesh.to_obj(tmp_obj_path)

        tmp_obj_path = self._handle_concavity(tmp_obj_path, self._cache_dir.name, concavity, mass)
        pyb_body_id = self.body_from_obj(tmp_obj_path, concavity=concavity, mass=mass)

        # Record the body id in the dictionary
        assert not pyb_body_id == -1, "Error in creating rigid body in PyBullet. Returning ID is -1."
        self.rigid_bodies_puids[name] = [pyb_body_id]

    def remove_rigid_body(self, name):
        raise NotImplementedError("remove_rigid_body is not implemented yet.")

    def pybullet_load_urdf(self, urdf_file):
        # type: (str) -> int
        """Instruct PyBullet to load a URDF file.

        Parameters
        ----------
        urdf_file : :obj:`str`
            The file path to the URDF file.

        Returns
        -------
        :obj:`int`
            The internal PyBullet unique id of the loaded URDF model."""

        with redirect_stdout(enabled=not self.verbose):
            # From PyBullet Quickstart Guide:
            # URDF_USE_INERTIA_FROM_FILE: by default, Bullet recomputed the inertia tensor based on mass and volume of the collision shape.
            # If you can provide more accurate inertia tensor, use this flag.
            # URDF_USE_SELF_COLLISION: by default, Bullet disables self-collision.
            # This flag let's you enable it.
            # You can customize the self-collision behavior using the following flags:
            # URDF_USE_SELF_COLLISION_EXCLUDE_PARENT will discard self-collision between links that are directly connected (parent and child).
            # URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS will discard self-collisions between a child link and any of its ancestors (parents, parents of parents, up to the base).

            # This function will use URDF_USE_SELF_COLLISION without the EXCLUDE_PARENT,
            # the neighboring self-collision situation is micro-managed by planning algorithms.

            # useFixedBase is set to false for allowing tool_models to move around.

            flags = pybullet.URDF_USE_SELF_COLLISION
            pybullet_uid = pybullet.loadURDF(urdf_file, useFixedBase=False, physicsClientId=self.client_id, flags=flags)

            # loadURDF returns a body unique id, a non-negative integer value. If the URDF file cannot be
            # loaded, this integer will be negative and not a valid body unique id.
            assert pybullet_uid >= 0, "Pybullet Error in loading URDF file {}".format(urdf_file)

            return pybullet_uid

    def robot_model_to_urdf(self, robot_model, concavity=False):
        """Converts a robot_model to a URDF package.

        Parameters
        ----------
        robot_model : :class:`compas_fab.robots.Robot`
            The robot_model to be saved for use with PyBullet.
        concavity : :obj:`bool`
            When ``False`` (the default), the mesh will be loaded as its
            convex hull for collision checking purposes.  When ``True``,
            a non-static mesh will be decomposed into convex parts using v-HACD.

        Returns
        -------
        :obj:`str`
            The file path to the robot model URDF package.

        Raises
        ------
        :exc:`Exception`
            If geometry has not been loaded.

        """
        robot_model.ensure_geometry()
        mesh_precision = 12

        for link in robot_model.links:
            # Assign a default material if none is provided.
            # Otherwise, the default material in PyBullet is black and is completely illegible.
            for visual in link.visual:
                if not visual.material:
                    visual.material = Material(name="undefined_gray", color=Color(0.7, 0.7, 0.7, alpha=1.0))

            # Export meshes as obj and handle concavity
            for element in itertools.chain(link.visual, link.collision):
                shape = element.geometry.shape
                if isinstance(shape, MeshDescriptor):
                    # Note: the MeshDescriptor.meshes object supports a list of compas meshes.
                    #       However URDF `link/mesh` tag only supports one filename to hold all the meshes.
                    #       Therefore, if there are multiple meshes in one Visual or Collision tag, the meshes
                    #       will be saved as one mesh file.
                    #       One the other hand, URDF allows multiple visual and collision tags under the same
                    #       link object.

                    # Join multiple meshes
                    if len(shape.meshes) > 1:
                        joined_mesh = Mesh()
                        for mesh in shape.meshes:
                            joined_mesh.join(mesh, False, precision=mesh_precision)
                        print("robot_model_to_urdf(): Joined %i meshes for link: " % len(shape.meshes), link.name)
                    else:
                        joined_mesh = shape.meshes[0]

                    # Export the joined mesh
                    mesh_file_name = str(joined_mesh.guid) + ".obj"
                    fp = os.path.join(self._cache_dir.name, mesh_file_name)
                    joined_mesh.to_obj(fp, precision=mesh_precision)
                    fp = self._handle_concavity(fp, self._cache_dir.name, concavity, 1, str(joined_mesh.guid))
                    shape.filename = fp

        # create urdf with new mesh locations
        urdf = URDF.from_robot(robot_model)

        # write urdf
        urdf_file_name = str(robot_model.guid) + ".urdf"
        urdf_filepath = os.path.join(self._cache_dir.name, urdf_file_name)

        # Note: Set prettify to False will cause some URDF to fail, such as rfl and panda.
        # Resulting in the non-debuggable "pybullet.error: Cannot load URDF file."
        # There is probably a bug in the XMLWriter or PyBullet's URDF parser.
        urdf.to_file(urdf_filepath, prettify=True)

        return urdf_filepath

    # --------------------------------
    # Functions for collision checking
    # --------------------------------

    def _check_collision(self, body_1_id, body_1_name, body_2_id, body_2_name, link_index_1=None, link_index_2=None):
        # type: (int, str, int, str, int, int) -> None
        """Internal low-level API to interface with Pybullet for collision checking.

        Parameters
        ----------
        body_1_id : :obj:`int`
            The unique id (issued by Pybullet) of the first body.
        body_1_name : :obj:`str`
            The name of the first body. Used for logging and error reporting only.
        body_2_id : :obj:`int`
            The unique id (issued by Pybullet) of the second body.
        body_2_name : :obj:`str`
            The name of the second body. Used for logging and error reporting only.
        link_index_1 : :obj:`int`, optional
            The link index if the first body is a robot. Defaults to ``None``.
        link_index_2 : :obj:`int`, optional
            The link index if the second body is a robot. Defaults to ``None``.
        """
        kwargs = {
            "bodyA": body_1_id,
            "bodyB": body_2_id,
            "distance": 0,
            "physicsClientId": self.client_id,
            "linkIndexA": link_index_1,
            "linkIndexB": link_index_2,
        }
        kwargs = {key: value for key, value in kwargs.items() if value is not None}
        pts = pybullet.getClosestPoints(**kwargs)
        if pts:
            message = "Collision between '{}' and '{}'".format(body_1_name, body_2_name)
            raise CollisionCheckError(message)

    # --------------------------------
    # Functions related to puids
    # --------------------------------

    def get_pose_joint_names_and_puids(self):
        # type: () -> List[str]
        """Returns the robot joints names and their puids that are need to create a pose in PyBullet.
        This include all joints that are not FIXED.
        Mimic joints are also included.

        The names and puids are ordered by PyBullet's unique ids.

        Returns
        -------
        :obj:`list` of :obj:`tuple` of :obj:`str`, :obj:`int`
            A list of tuples containing the joint names and their PyBullet unique ids.
        """
        joint_names_and_puids = []
        robot_puid = self.robot_puid
        for puid in range(self._get_num_joints(robot_puid)):
            joint_name = self._get_joint_name(puid, robot_puid)
            joint_type = self._get_joint_type(puid, robot_puid)
            if joint_type != const.JOINT_FIXED:
                joint_names_and_puids.append((joint_name, puid))
        return joint_names_and_puids

    def build_pose_for_pybullet(self, configuration):
        # type: (Configuration) -> List[float]
        """Builds a robot pose (list of joint values) for sending to PyBullet.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The configuration to be converted.

        Returns
        -------
        :obj:`list` of :obj:`float`
            A list of joint values.
        """
        joint_names_and_puids = self.get_pose_joint_names_and_puids()

        joint_values = []
        for joint_name, joint_puid in joint_names_and_puids:
            if joint_name in configuration:
                joint_values.append(configuration[joint_name])
            else:
                # Check if this is mimic joint
                joint = self.robot.model.get_joint_by_name(joint_name)
                mimic = joint.mimic  # type: Mimic
                # Get the value of the joint that is being mimicked (works only for non-cascaded mimic)
                if mimic:
                    mimicked_joint_position = configuration[mimic.joint]
                    joint_values.append(mimic.calculate_position(mimicked_joint_position))
                else:
                    raise ValueError(
                        "Joint value for '{}' is needed for Pybullet but not found in the provided configuration.".format(
                            joint_name
                        )
                    )
        return joint_values

    def _get_base_frame(self, body_id):
        pose = pybullet.getBasePositionAndOrientation(body_id, physicsClientId=self.client_id)
        return frame_from_pose(pose)

    def _get_base_name(self, body_id):
        return self._get_body_info(body_id).base_name.decode(encoding="UTF-8")

    def _get_link_state(self, link_id, body_id):
        return const.LinkState(
            *pybullet.getLinkState(body_id, link_id, computeForwardKinematics=True, physicsClientId=self.client_id)
        )

    def _get_joint_state(self, joint_id, body_id):
        return const.JointState(*pybullet.getJointState(body_id, joint_id, physicsClientId=self.client_id))

    def _get_joint_states(self, joint_ids, body_id):
        return [
            const.JointState(*js) for js in pybullet.getJointStates(body_id, joint_ids, physicsClientId=self.client_id)
        ]

    def _get_body_info(self, body_id):
        return const.BodyInfo(*pybullet.getBodyInfo(body_id, physicsClientId=self.client_id))

    def _get_joint_info(self, joint_id, body_id):
        return const.JointInfo(*pybullet.getJointInfo(body_id, joint_id, physicsClientId=self.client_id))

    def _get_num_joints(self, body_id):
        return pybullet.getNumJoints(body_id, physicsClientId=self.client_id)

    def _get_joint_name(self, joint_id, body_id):
        return self._get_joint_info(joint_id, body_id).jointName.decode("UTF-8")

    def _get_joint_type(self, joint_id, body_id):
        return self._get_joint_info(joint_id, body_id).jointType

    def _get_link_name(self, link_id, body_id):
        if link_id == const.BASE_LINK_ID:
            return self._get_base_name(body_id)
        return self._get_joint_info(link_id, body_id).linkName.decode("UTF-8")

    # ----------------------------------------
    # Functions for configuration and frames
    # ----------------------------------------

    def _get_link_frame(self, link_id, body_id):
        if link_id == const.BASE_LINK_ID:
            return self._get_base_frame(body_id)
        link_state = self._get_link_state(link_id, body_id)
        pose = (link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation)
        return frame_from_pose(pose)

    def _set_base_frame(self, frame, body_id):
        point, quaternion = pose_from_frame(frame)
        pybullet.resetBasePositionAndOrientation(body_id, point, quaternion, physicsClientId=self.client_id)

    def _set_joint_position(self, joint_id, value, body_id):
        pybullet.resetJointState(body_id, joint_id, value, targetVelocity=0, physicsClientId=self.client_id)

    def _set_joint_positions(self, joint_ids, values, body_id):
        if len(joint_ids) != len(values):
            raise Exception("Joints and values must have the same length.")
        for joint_id, value in zip(joint_ids, values):
            self._set_joint_position(joint_id, value, body_id)

    def _get_joint_positions(self, joint_ids, body_id):
        # type: (List[int], int) -> List[float]
        """Returns the joint positions of any robot-model-like object in the PyBullet server."""
        joint_states = self._get_joint_states(joint_ids, body_id)
        return [js.jointPosition for js in joint_states]

    def set_robot_configuration(self, configuration):
        # type: (Configuration) -> None
        """Sets the robot's pose to the given configuration.

        Only the joint values in the configuration are set.
        The other joint values are not changed.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The configuration to be set, ``joint_names`` must be included in the configuration.
        group : :obj:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.

        """
        # Check if the robot has been loaded, careful, puid can be zero
        assert (
            self.robot_puid is not None
        ), "PyBulletClient.robot_puid is None. Robot must be loaded before setting configuration."

        assert (
            configuration.joint_names != []
        ), "Joint names must be provided in the configuration passed to set_robot_configuration."

        # Iterate through all joints that are considered free by PyBullet
        for joint_name, joint_puid in self.get_pose_joint_names_and_puids():
            if joint_name in configuration:
                self._set_joint_position(joint_puid, configuration[joint_name], self.robot_puid)
            else:
                # Check if this is mimic joint
                joint = self.robot.model.get_joint_by_name(joint_name)
                mimic = joint.mimic  # type: Mimic
                # Get the value of the joint that is being mimicked (works only for non-cascaded mimic)
                if mimic:
                    if mimic.joint in configuration:
                        mimicked_joint_position = configuration[mimic.joint]
                        self._set_joint_position(
                            joint_puid, mimic.calculate_position(mimicked_joint_position), self.robot_puid
                        )
                    # Note: If the joint that is being mimicked is not in the configuration, the mimic joint will not be set.
                    # This search and replace can be more elaborate in the future if needed.

    def get_robot_configuration(self):
        # type: () -> Configuration
        """Gets the robot's current pose.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`  The robot to be configured.

        Returns
        -------
        :class:`compas_robots.Configuration`
        """
        robot = self.robot_cell.robot
        configuration = robot.zero_configuration()
        for joint_name in configuration.joint_names:
            joint_id = self.robot_joint_puids[joint_name]
            joint_value = self._get_joint_positions([joint_id], self.robot_puid)[0]
            configuration[joint_name] = joint_value

        return configuration

    def set_tool_base_frame(self, tool_name, frame):
        # type: (str, Frame) -> None
        """Sets the base frame of a tool.

        Parameters
        ----------
        tool_name : :obj:`str`
            Name of the tool.
        frame : :class:`compas.geometry.Frame`
            The frame to which the tool should be moved.
        """
        tool_id = self.tools_puids[tool_name]
        self.set_object_frame(tool_id, frame)

    def set_rigid_body_base_frame(self, rigid_body_name, frame):
        # type: (str, Frame) -> None
        """Sets the base frame of a rigid body.

        Parameters
        ----------
        rigid_body_name : :obj:`str`
            Name of the rigid body.
        frame : :class:`compas.geometry.Frame`
            The frame to which the rigid body should be moved.
        """
        body_ids = self.rigid_bodies_puids[rigid_body_name]
        for body_id in body_ids:
            self.set_object_frame(body_id, frame)

    def set_object_frame(self, body_id, frame):
        (point, quat) = pose_from_frame(frame)
        pybullet.resetBasePositionAndOrientation(body_id, point, quat, physicsClientId=self.client_id)

    # ------------------------------------------------------------------------------------
    # Helper functions for creating rigid bodies in PyBullet
    # This includes loading meshes via OBJ files, setting up visual and collision objects
    # ------------------------------------------------------------------------------------

    def convert_mesh_to_body(self, mesh, frame, concavity=False, mass=const.STATIC_MASS):
        """Creates a pybullet body from a compas mesh and attaches it to the scene.

        Useful for static collision meshes and attached collision meshes.
        The mesh is first exported to an OBJ file and then loaded into PyBullet.
        When concavity is requested, the mesh will be decomposed into convex parts.

        This function should not be called directly by compas_fab API user because PyBulletClient
        will otherwise not be able to keep track of the collision objects.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
            The mesh to be converted, modelled relative to its own object coordinate system.
        frame : :class:`compas.geometry.Frame`
            The frame to which the mesh should be moved.
        concavity : :obj:`bool`, optional
            When ``False`` (the default), the mesh will be loaded as is and any convex hull will be considered solid during collision checking.
            When ``True``, the mesh will be decomposed into convex parts using V-HACD within PyBullet.
        mass : :obj:`float`, optional
            Mass of the body to be created, in kg.  If ``0`` mass is given (the default),
            the object is static.

        Returns
        -------
        :obj:`int`
            The PyBullet body id of the mesh.

        Notes
        -----
        If this method is called several times with the same ``mesh`` instance, but the ``mesh`` has been modified
        in between calls, PyBullet's default caching behavior will prevent it from recognizing these changes.  It
        is best practice to create a new mesh instance or to make use of the `frame` argument, if applicable.  If
        this is not possible, PyBullet's caching behavior can be changed with
        ``pybullet.setPhysicsEngineParameter(enableFileCaching=0)``.
        """
        tmp_obj_path = os.path.join(self._cache_dir.name, "{}.obj".format(mesh.guid))
        mesh.to_obj(tmp_obj_path)
        tmp_obj_path = self._handle_concavity(tmp_obj_path, self._cache_dir.name, concavity, mass)
        pyb_body_id = self.body_from_obj(tmp_obj_path, concavity=concavity, mass=mass)
        self._set_base_frame(frame, pyb_body_id)

        return pyb_body_id

    @staticmethod
    def _handle_concavity(tmp_obj_path, tmp_dir, concavity, mass, mesh_name=""):
        """Decompose a mesh into convex parts using v-HACD if concavity is requested.

        Parameters
        ----------
        tmp_obj_path : :obj:`str`
            Path to the OBJ file as input.
        tmp_dir : :obj:`str`
            Path to the temporary directory for storing the output.
        concavity : :obj:`bool`
            When ``False``, the mesh will not be decomposed and the input path will be returned.
        mass : :obj:`float`
            Mass of the body to be created, in kg.  If ``0`` mass is given, the object is static.
            Static objects will not be decomposed.
        mesh_name : :obj:`str`, optional
            A prefix for the temporary mesh file. Typically not required.
            Defaults to ``""``, which generates a temporary file named ``vhacd_temp.obj``.

        Returns
        -------
        :obj:`str`
            Path to the OBJ file as output.
        """
        if not concavity or mass == const.STATIC_MASS:
            return tmp_obj_path
        if mesh_name:
            mesh_name += "_"
        tmp_vhacd_obj_path = os.path.join(tmp_dir, mesh_name + "vhacd_temp.obj")
        tmp_log_path = os.path.join(tmp_dir, mesh_name + "log.txt")
        with redirect_stdout():
            pybullet.vhacd(tmp_obj_path, tmp_vhacd_obj_path, tmp_log_path)
        return tmp_vhacd_obj_path

    def body_from_obj(self, path, scale=1.0, concavity=False, mass=const.STATIC_MASS, collision=True, color=const.GREY):
        """Create a PyBullet body from an OBJ file.

        Parameters
        ----------
        path : :obj:`str`
            Path to the OBJ file.
        scale : :obj:`float`, optional
            Factor by which to scale the mesh. Defaults to ``1.``
        concavity : :obj:`bool`, optional
            When ``False`` (the default), the mesh will be loaded as its convex hull for collision checking purposes.
            Only applicable to static (massless) objects. For non-static meshes, use OBJs preprocessed by
            ``pybullet.vhacd``.
        mass : :obj:`float`, optional
            Mass of the body to be created, in kg.  If `0` mass is given (the default),
            the object is static.
        collision : :obj:`bool`
            When ``True``, body will be included in collision checking calculations. Defaults to ``True``.
        color : :obj:`tuple` of :obj:`float`
            RGBa color components of the body.

        Returns
        -------
        :obj:`int`
        """
        concavity &= mass == const.STATIC_MASS
        geometry_args = self._get_geometry_args(path, concavity=concavity, scale=scale)

        collision_args = self._get_collision_args(geometry_args)
        collision_id = self._create_collision_shape(collision_args) if collision else const.NULL_ID

        visual_args = self._get_visual_args(geometry_args, color=color)
        visual_id = self._create_visual_shape(visual_args)

        body_id = self._create_body(collision_id, visual_id, mass=mass)
        return body_id

    def _create_body(self, collision_id=const.NULL_ID, visual_id=const.NULL_ID, mass=const.STATIC_MASS):
        # type: (int, int, float) -> int
        """Create a PyBullet MultiBodies from a collision and visual shape.

        Refer to `btMultiBody` in Bullet documentation and `createMultiBody` in PyBullet documentation.

        Returns
        -------
        :obj:`int`
            A single PyBullet body id for the multi-body object.
        """
        return pybullet.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            physicsClientId=self.client_id,
        )

    @staticmethod
    def _create_collision_shape(collision_args):
        return pybullet.createCollisionShape(**collision_args)

    @staticmethod
    def _create_visual_shape(visual_args):
        if visual_args.get("rgbaColor") is None:
            return const.NULL_ID
        return pybullet.createVisualShape(**visual_args)

    def _get_visual_args(self, geometry_args, frame=Frame.worldXY(), color=const.RED, specular=None):
        point, quaternion = pose_from_frame(frame)
        visual_args = {
            "rgbaColor": color,
            "visualFramePosition": point,
            "visualFrameOrientation": quaternion,
            "physicsClientId": self.client_id,
        }
        visual_args.update(geometry_args)
        if specular is not None:
            visual_args["specularColor"] = specular
        return visual_args

    def _get_collision_args(self, geometry_args, frame=Frame.worldXY()):
        point, quaternion = pose_from_frame(frame)
        collision_args = {
            "collisionFramePosition": point,
            "collisionFrameOrientation": quaternion,
            "physicsClientId": self.client_id,
        }
        collision_args.update(geometry_args)
        if "length" in collision_args:
            # pybullet bug visual => length, collision => height
            collision_args["height"] = collision_args["length"]
            del collision_args["length"]
        return collision_args

    @staticmethod
    def _get_geometry_args(path, concavity=False, scale=1.0):
        geometry_args = {
            "shapeType": pybullet.GEOM_MESH,
            "fileName": path,
            "meshScale": [scale] * 3,
        }
        if concavity:
            geometry_args["flags"] = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
        return geometry_args


class AnalyticalPyBulletClient(PyBulletClient):
    """Combination of PyBullet as the client for Collision Detection and Analytical Inverse Kinematics."""

    def __init__(self, connection_type="gui", verbose=False):
        super(AnalyticalPyBulletClient, self).__init__(connection_type=connection_type, verbose=verbose)

    # TODO: Move the following function to AnalyticalPyBulletPlanner
    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        planner = AnalyticalInverseKinematics(self)
        return planner.inverse_kinematics(robot, frame_WCF, start_configuration, group, options)

    def plan_cartesian_motion(self, robot, waypoints, start_configuration=None, group=None, options=None):
        planner = AnalyticalPlanCartesianMotion(self)
        return planner.plan_cartesian_motion(robot, waypoints, start_configuration, group, options)
