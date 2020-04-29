from __future__ import print_function

import logging

from compas_fab.backends.client import ClientInterface
from compas_fab.backends.vrep import VrepError
from compas_fab.backends.vrep.helpers import assert_robot
from compas_fab.backends.vrep.helpers import config_from_vrep
from compas_fab.backends.vrep.helpers import config_to_vrep
from compas_fab.backends.vrep.helpers import floats_from_vrep
from compas_fab.backends.vrep.helpers import floats_to_vrep
from compas_fab.backends.vrep.helpers import resolve_host
from compas_fab.backends.vrep.planner_backend_vrep import VrepPlanner
from compas_fab.backends.vrep.remote_api import vrep

DEFAULT_SCALE = 1.
DEFAULT_OP_MODE = vrep.simx_opmode_blocking
CHILD_SCRIPT_TYPE = vrep.sim_scripttype_childscript
LOG = logging.getLogger('compas_fab.backends.vrep.client')

__all__ = [
    'VrepClient',
]


class VrepClient(ClientInterface):
    """Interface to run simulations using VREP as
    the engine for kinematics and path planning.

    :class:`.VrepClient` is a context manager type, so it's best used in combination
    with the ``with`` statement to ensure resource deallocation.


    Args:
        host (:obj:`str`): IP address or DNS name of the V-REP simulator.
        port (:obj:`int`): Port of the V-REP simulator.
        scale (:obj:`int`): Scaling of the model. Defaults to meters (``1``).
        lua_script (:obj:`str`): Name of the LUA script on the V-REP scene.
        debug (:obj:`bool`): True to enable debug messages, False otherwise.

    Examples:

        >>> from compas_fab.backends import VrepClient
        >>> with VrepClient() as client:
        ...     print ('Connected: %s' % client.is_connected)
        Connected: True

    Note:
        For more examples, check out the :ref:`V-REP examples page <examples_vrep>`.
    """

    def __init__(self, host='127.0.0.1', port=19997, scale=DEFAULT_SCALE, lua_script='RFL', debug=False):
        super(VrepClient, self).__init__()
        self.client_id = None
        self.host = resolve_host(host)
        self.port = port
        self.default_timeout_in_ms = -50000000
        self.thread_cycle_in_ms = 5
        self.debug = debug
        self.scale = float(scale)
        self.lua_script = lua_script
        self._added_handles = []

        self.planner = VrepPlanner(self)

    def __enter__(self):
        # Stop existing simulation, if any
        vrep.simxFinish(-1)

        if self.debug:
            LOG.debug('Connecting to V-REP on %s:%d...', self.host, self.port)

        # Connect to V-REP, set a very large timeout for blocking commands
        self.client_id = vrep.simxStart(self.host, self.port, True, True,
                                        self.default_timeout_in_ms,
                                        self.thread_cycle_in_ms)

        # Start simulation
        vrep.simxStartSimulation(self.client_id, DEFAULT_OP_MODE)

        if self.client_id == -1:
            raise VrepError('Unable to connect to V-REP on %s:%d' % (self.host, self.port), -1)

        return self

    def __exit__(self, *args):
        # Stop simulation
        vrep.simxStopSimulation(self.client_id, DEFAULT_OP_MODE)

        # Close the connection to V-REP
        vrep.simxFinish(self.client_id)
        self.client_id = None

        # Objects are removed by V-REP itself when simulation stops
        self._added_handles = []

        if self.debug:
            LOG.debug('Disconnected from V-REP')

    @property
    def is_connected(self):
        """Indicates whether the client has an active connection.

        Returns:
            bool: True if connected, False otherwise.
        """
        return self.client_id is not None and self.client_id != -1

    def get_object_handle(self, object_name):
        """Gets the object handle (identifier) for a given object name.

        Args:
            object_name (:obj:`str`): Name of the object.

        Returns:
            int: Object handle.
        """
        _res, handle = vrep.simxGetObjectHandle(self.client_id,
                                                object_name,
                                                DEFAULT_OP_MODE)
        return handle

    def get_object_matrices(self, object_handles):
        """Gets a dictionary of matrices keyed by object handle.

        Args:
            object_handles (:obj:`list` of :obj:`float`): List of object handles (identifiers)
                to retrieve matrices from.

        Returns:
            dict: Dictionary of matrices represented by a :obj:`list` of 12 :obj:`float` values.

        Examples:

            >>> from compas_fab.backends import VrepClient
            >>> with VrepClient() as client:
            ...     matrices = client.get_object_matrices([0])
            ...     print([int(i) for i in matrices[0]])   # doctest: +SKIP
            [0, 0, 0, 19, 0, 0, 0, 10, 0, 0, 0, 6]         # doctest: +SKIP

        .. note::
            The resulting dictionary is keyed by object handle.
        """
        _res, _, matrices, _, _ = self.run_child_script('getShapeMatrices', object_handles, [], [])
        return dict([(object_handles[i // 12], floats_from_vrep(matrices[i:i + 12], self.scale)) for i in range(0, len(matrices), 12)])

    def get_all_visible_handles(self):
        """Gets a list of object handles (identifiers) for all visible
        shapes of the 3D model.

        Returns:
            list: List of object handles (identifiers) of the 3D model.
        """
        return self.run_child_script('getRobotVisibleShapeHandles', [], [], [])[1]

    def set_robot_metric(self, group, metric_values):
        """Assigns a metric defining relations between axis values of a robot.

        It takes a list containing one value per configurable joint. Each value
        ranges from 0 to 1, where 1 indicates the axis is blocked and cannot
        move during inverse kinematic solving. A value of 1 on any of these
        effectively removes one degree of freedom (DOF).

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1.
        """
        vrep.simxCallScriptFunction(self.client_id,
                                    self.lua_script,
                                    CHILD_SCRIPT_TYPE, 'setTheMetric',
                                    [group], metric_values, [],
                                    bytearray(), DEFAULT_OP_MODE)

    def set_robot_pose(self, robot, frame):
        """Moves the robot to a given pose, specified as a frame.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance to move.
            frame (:class:`Frame`): Target or goal frame.

        Returns:
            An instance of :class:`Configuration` found for the given pose.
        """
        assert_robot(robot)

        # First check if the start state is reachable
        joints = len(robot.get_configurable_joints())
        options = {
            'num_joints': joints,
            'metric_values': [0.] * joints,
        }
        config = self.inverse_kinematics(frame, group=robot.model.attr['index'], options=options)[-1]

        if not config:
            raise ValueError('Cannot find a valid config for the given pose')

        self.set_robot_config(robot, config)

        return config

    def set_robot_config(self, robot, config):
        """Moves the robot to the specified configuration.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance to move.
            config (:class:`Configuration` instance): Describes the position of the
                robot as an instance of :class:`Configuration`.

        Examples:

            >>> from compas_fab.robots import *
            >>> with VrepClient() as client:
            ...     config = Configuration.from_prismatic_and_revolute_values([7.600, -4.500, -5.500],
            ...                                                               to_radians([90, 0, 0, 0, 0, -90]))
            ...     client.set_robot_config(rfl.Robot('A'), config)
            ...
        """
        assert_robot(robot)

        if not config:
            raise ValueError('Unsupported config value')

        values = config_to_vrep(config, self.scale)

        self.set_robot_metric(robot.model.attr['index'], [0.0] * len(config.values))
        self.run_child_script('moveRobotFK',
                              [], values, ['robot' + robot.name])

    def get_robot_config(self, robot):
        """Gets the current configuration of the specified robot.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance.

        Examples:

            >>> from compas_fab.robots import *
            >>> with VrepClient() as client:
            ...     config = client.get_robot_config(rfl.Robot('A'))

        Returns:
            An instance of :class:`.Configuration`.
        """
        assert_robot(robot)

        _res, _, config, _, _ = self.run_child_script('getRobotState',
                                                      [robot.model.attr['index']],
                                                      [], [])
        return config_from_vrep(config, self.scale)

    def find_raw_robot_states(self, group, goal_vrep_pose, gantry_joint_limits, arm_joint_limits, max_trials=None, max_results=1):
        i = 0
        final_states = []
        retry_until_success = True if not max_trials else False

        while True:
            string_param_list = []
            if gantry_joint_limits or arm_joint_limits:
                joint_limits = []
                joint_limits.extend(floats_to_vrep(gantry_joint_limits or [], self.scale))
                joint_limits.extend(arm_joint_limits or [])
                string_param_list.append(','.join(map(str, joint_limits)))

            res, _, states, _, _ = self.run_child_script('searchRobotStates',
                                                         [group,
                                                          max_trials or 1,
                                                          max_results],
                                                         goal_vrep_pose, string_param_list)

            # Even if the retry_until_success is set to True, we short circuit
            # at some point to prevent infinite loops caused by misconfiguration
            i += 1
            if i > 20 or (res != 0 and not retry_until_success):
                raise VrepError('Failed to search robot states', res)

            final_states.extend(states)

            if len(final_states):
                LOG.info('Found %d valid robot states', len(final_states) // 9)
                break
            else:
                LOG.info('No valid robot states found, will retry.')

        return final_states

    def pick_building_member(self, robot, building_member_mesh, pickup_frame, metric_values=None):
        """Picks up a building member and attaches it to the robot.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance to use for pick up.
            building_member_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.
            pickup_pose (:class:`Frame`): Pickup frame.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.

        Returns:
            int: Object handle (identifier) assigned to the building member.
        """
        assert_robot(robot)

        joints = len(robot.get_configurable_joints())
        if not metric_values:
            metric_values = [0.1] * joints

        self.set_robot_pose(robot, pickup_frame)

        return self.add_building_member(robot, building_member_mesh)

    def add_building_member(self, robot, building_member_mesh):
        """Adds a building member to the 3D scene and attaches it to the robot.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance to attach the building member to.
            building_member_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.

        Returns:
            int: Object handle (identifier) assigned to the building member.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        assert_robot(robot)

        handles = self.add_meshes([building_member_mesh])

        if len(handles) != 1:
            raise VrepError('Expected one handle, but multiple found=' + str(handles), -1)

        handle = handles[0]

        parent_handle = self.get_object_handle('customGripper' + robot.name + '_connection')
        vrep.simxSetObjectParent(self.client_id, handle, parent_handle, True, DEFAULT_OP_MODE)

        return handle

    def add_meshes(self, meshes):
        """Adds meshes to the 3D scene.

        Args:
            meshes (:obj:`list` of :class:`compas.datastructures.Mesh`): List
                of meshes to add to the current simulation scene.

        Returns:
            list: List of object handles (identifiers) assigned to the meshes.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        mesh_handles = []

        for mesh in meshes:
            if not mesh.is_trimesh():
                raise ValueError('The V-REP client only supports tri-meshes')

            vertices, faces = mesh.to_vertices_and_faces()
            vrep_packing = (floats_to_vrep([item for sublist in vertices for item in sublist], self.scale) +
                            [item for sublist in faces for item in sublist])
            params = [[len(vertices) * 3, len(faces) * 4], vrep_packing]
            handles = self.run_child_script('buildMesh',
                                            params[0],
                                            params[1],
                                            [])[1]
            mesh_handles.extend(handles)
            self._added_handles.extend(handles)

        return mesh_handles

    def remove_meshes(self, mesh_handles):
        """Removes meshes from the 3D scene.

        This is functionally identical to ``remove_objects``, but it's here for
        symmetry reasons.

        Args:
            mesh_handles (:obj:`list` of :obj:`int`): Object handles to remove.
        """
        self.remove_objects(mesh_handles)

    def remove_objects(self, object_handles):
        """Removes objects from the 3D scene.

        Args:
            object_handles (:obj:`list` of :obj:`int`): Object handles to remove.

        .. note::
            Please note there's no need to clean up objects manually after the simulation
            has completed, as those will be reset automatically anyway. This method is
            only useful if you need to remove objects *during* a simulation.
        """
        for handle in object_handles:
            vrep.simxRemoveObject(self.client_id, handle, DEFAULT_OP_MODE)

        self._added_handles = filter(lambda x: x not in object_handles, self._added_handles)

    def run_child_script(self, function_name, in_ints, in_floats, in_strings):
        return vrep.simxCallScriptFunction(self.client_id,
                                           self.lua_script,
                                           CHILD_SCRIPT_TYPE, function_name,
                                           in_ints, in_floats, in_strings,
                                           bytearray(), DEFAULT_OP_MODE)
