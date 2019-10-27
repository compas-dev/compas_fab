from __future__ import print_function

import logging
import math
import socket
from timeit import default_timer as timer

from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import matrix_from_frame

from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.vrep.remote_api import vrep
from compas_fab.robots import Configuration
from compas_fab.robots import Robot

DEFAULT_SCALE = 1.
DEFAULT_OP_MODE = vrep.simx_opmode_blocking
CHILD_SCRIPT_TYPE = vrep.sim_scripttype_childscript
LOG = logging.getLogger('compas_fab.backends.vrep.client')

__all__ = [
    'VrepError',
    'VrepClient',
]


class VrepError(BackendError):
    """Wraps an exception that occurred inside the simulation engine."""

    def __init__(self, message, error_code):
        super(VrepError, self).__init__('Error code: ' +
                                        str(error_code) +
                                        '; ' + message)
        self.error_code = error_code


class VrepClient(object):
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
    SUPPORTED_PLANNERS = ('bitrrt', 'bkpiece1', 'est', 'kpiece1',
                           'lazyprmstar', 'lbkpiece1', 'lbtrrt', 'pdst',
                           'prm', 'prrt', 'rrt', 'rrtconnect', 'rrtstar',
                           'sbl', 'stride', 'trrt')

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

    def set_robot_metric(self, robot, metric_values):
        """Assigns a metric defining relations between axis values of a robot.

        It takes a list containing one value per configurable joint. Each value
        ranges from 0 to 1, where 1 indicates the axis is blocked and cannot
        move during inverse kinematic solving. A value of 1 on any of these
        effectively removes one degree of freedom (DOF).

        Args:
            robot (:class:`Robot`): Robot instance.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1.
        """
        assert_robot(robot)

        vrep.simxCallScriptFunction(self.client_id,
                                    self.lua_script,
                                    CHILD_SCRIPT_TYPE, 'setTheMetric',
                                    [robot.model.attr['index']], metric_values, [],
                                    bytearray(), DEFAULT_OP_MODE)

    def set_robot_pose(self, robot, frame):
        """Moves the robot to a given pose, specified as a frame.

        Args:
            robot (:class:`Robot`): Robot instance to move.
            frame (:class:`Frame`): Target or goal frame.

        Returns:
            An instance of :class:`Configuration` found for the given pose.
        """
        assert_robot(robot)

        # First check if the start state is reachable
        joints = len(robot.get_configurable_joints())
        config = self.inverse_kinematics(robot, frame, [0.] * joints)[-1]

        if not config:
            raise ValueError('Cannot find a valid config for the given pose')

        self.set_robot_config(robot, config)

        return config

    def set_robot_config(self, robot, config):
        """Moves the robot to the specified configuration.

        Args:
            robot (:class:`Robot`): Robot instance to move.
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

        self.set_robot_metric(robot, [0.0] * len(config.values))
        self.run_child_script('moveRobotFK',
                              [], values, ['robot' + robot.name])

    def get_robot_config(self, robot):
        """Gets the current configuration of the specified robot.

        Args:
            robot (:class:`Robot`): Robot instance.

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

    def forward_kinematics(self, robot):
        """Calculates forward kinematics to get the current end-effector pose.

        Args:
            robot (:class:`Robot`): Robot instance.

        Examples:

            >>> from compas_fab.robots import *
            >>> with VrepClient() as client:
            ...     frame = client.forward_kinematics(rfl.Robot('A'))

        Returns:
            An instance of :class:`Frame`.
        """
        assert_robot(robot)

        _res, _, pose, _, _ = self.run_child_script('getIkTipPose',
                                                     [robot.model.attr['index']],
                                                     [], [])
        return vrep_pose_to_frame(pose, self.scale)

    def inverse_kinematics(self, robot, goal_frame, metric_values=None, gantry_joint_limits=None, arm_joint_limits=None, max_trials=None, max_results=1):
        """Calculates inverse kinematics to find valid robot configurations for the specified goal frame.

        Args:
            robot (:class:`Robot`): Robot instance.
            goal_frame (:class:`Frame`): Target or goal frame.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.
            gantry_joint_limits (:obj:`list` of `float`): List of 6 floats defining the upper/lower limits of
                gantry joints. Use this if you want to restrict the area in which to search for states.
            arm_joint_limits (:obj:`list` of `float`): List of 12 floats defining the upper/lower limits of
                arm joints. Use this if you want to restrict the working area in which to search for states.
            max_trials (:obj:`int`): Number of trials to run. Set to ``None``
                to retry infinitely.
            max_results (:obj:`int`): Maximum number of result states to return.

        Returns:
            list: List of :class:`Configuration` objects representing
            the collision-free configuration for the ``goal_frame``.
        """
        assert_robot(robot)

        joints = len(robot.get_configurable_joints())
        if not metric_values:
            metric_values = [0.1] * joints

        self.set_robot_metric(robot, metric_values)

        states = self._find_raw_robot_states(robot, frame_to_vrep_pose(goal_frame, self.scale), gantry_joint_limits, arm_joint_limits, max_trials, max_results)

        return [config_from_vrep(states[i:i + joints], self.scale)
                for i in range(0, len(states), joints)]

    def _find_raw_robot_states(self, robot, goal_vrep_pose, gantry_joint_limits, arm_joint_limits, max_trials=None, max_results=1):
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
                                                         [robot.model.attr['index'],
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
            robot (:class:`Robot`): Robot instance to use for pick up.
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

    def _find_path_plan(self, robot, goal, metric_values, collision_meshes,
                        planner_id, trials, resolution,
                        gantry_joint_limits, arm_joint_limits, shallow_state_search, optimize_path_length):

        joints = len(robot.get_configurable_joints())
        if not metric_values:
            metric_values = [0.1] * joints

        if planner_id not in self.SUPPORTED_PLANNERS:
            raise ValueError('Unsupported planner_id. Must be one of: ' + str(self.SUPPORTED_PLANNERS))

        first_start = timer() if self.debug else None
        if collision_meshes:
            self.add_meshes(collision_meshes)
        if self.debug:
            LOG.debug('Execution time: add_meshes=%.2f', timer() - first_start)

        start = timer() if self.debug else None
        self.set_robot_metric(robot, metric_values)
        if self.debug:
            LOG.debug('Execution time: set_robot_metric=%.2f', timer() - start)

        if 'target_type' not in goal:
            raise ValueError('Invalid goal type, you are using an internal function but passed incorrect args')

        if goal['target_type'] == 'config':
            states = []
            for c in goal['target']:
                states.extend(config_to_vrep(c, self.scale))
        elif goal['target_type'] == 'pose':
            start = timer() if self.debug else None
            max_trials = None if shallow_state_search else 80
            max_results = 1 if shallow_state_search else 80
            states = self._find_raw_robot_states(robot, frame_to_vrep_pose(goal['target'], self.scale), gantry_joint_limits, arm_joint_limits, max_trials, max_results)
            if self.debug:
                LOG.debug('Execution time: search_robot_states=%.2f', timer() - start)

        start = timer() if self.debug else None
        string_param_list = [planner_id]
        if gantry_joint_limits or arm_joint_limits:
            joint_limits = []
            joint_limits.extend(floats_to_vrep(gantry_joint_limits or [], self.scale))
            joint_limits.extend(arm_joint_limits or [])
            string_param_list.append(','.join(map(str, joint_limits)))

        if self.debug:
            LOG.debug('About to execute path planner: planner_id=%s, trials=%d, shallow_state_search=%s, optimize_path_length=%s',
                      planner_id, trials, shallow_state_search, optimize_path_length)

        res, _, path, _, _ = self.run_child_script('searchRobotPath',
                                                   [robot.model.attr['index'],
                                                    trials,
                                                    (int)(resolution * 1000),
                                                    1 if optimize_path_length else 0],
                                                   states, string_param_list)
        if self.debug:
            LOG.debug('Execution time: search_robot_path=%.2f', timer() - start)

        if res != 0:
            raise VrepError('Failed to search robot path', res)

        if self.debug:
            LOG.debug('Execution time: total=%.2f', timer() - first_start)

        return [config_from_vrep(path[i:i + joints], self.scale)
                for i in range(0, len(path), joints)]

    def plan_motion_to_config(self, robot, goal_configs, metric_values=None, collision_meshes=None,
                              planner_id='rrtconnect', trials=1, resolution=0.02,
                              gantry_joint_limits=None, arm_joint_limits=None, shallow_state_search=True, optimize_path_length=False):
        """Find a path plan to move the selected robot from its current position to one of the `goal_configs`.

        This function is useful when it is required to get a path plan that ends in one
        specific goal configuration.

        Args:
            robot (:class:`Robot`): Robot instance to move.
            goal_configs (:obj:`list` of :class:`Configuration`): List of target or goal configurations.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.
            collision_meshes (:obj:`list` of :class:`compas.datastructures.Mesh`): Collision meshes
                to be taken into account when calculating the motion plan.
                Defaults to ``None``.
            planner_id (:obj:`str`): Name of the planner to use. Defaults to ``rrtconnect``.
            trials (:obj:`int`): Number of search trials to run. Defaults to ``1``.
            resolution (:obj:`float`): Validity checking resolution. This value
                is specified as a fraction of the space's extent.
                Defaults to ``0.02``.
            gantry_joint_limits (:obj:`list` of `float`): List of 6 floats defining the upper/lower limits of
                gantry joints. Use this if you want to restrict the working area of the path planner.
            arm_joint_limits (:obj:`list` of `float`): List of 12 floats defining the upper/lower limits of
                arm joints. Use this if you want to restrict the working area of the path planner.
            shallow_state_search (:obj:`bool`): True to search only a minimum of
                valid states before searching a path, False to search states intensively.
            optimize_path_length (:obj:`bool`): True to search the path with minimal total length among all `trials`,
                False to return the first valid path found. It only affects the output if `trials > 1`.

        Returns:
            list: List of :class:`Configuration` objects representing the
            collision-free path to the ``goal_configs``.
        """
        assert_robot(robot)
        return self._find_path_plan(robot, {'target_type': 'config', 'target': goal_configs},
                                    metric_values, collision_meshes, planner_id, trials, resolution,
                                    gantry_joint_limits, arm_joint_limits, shallow_state_search, optimize_path_length)

    def plan_motion(self, robot, goal_frame, metric_values=None, collision_meshes=None,
                    planner_id='rrtconnect', trials=1, resolution=0.02,
                    gantry_joint_limits=None, arm_joint_limits=None, shallow_state_search=True, optimize_path_length=False):
        """Find a path plan to move the selected robot from its current position to the `goal_frame`.

        Args:
            robot (:class:`Robot`): Robot instance to move.
            goal_frame (:class:`Frame`): Target or goal frame.
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.
            collision_meshes (:obj:`list` of :class:`compas.datastructures.Mesh`): Collision meshes
                to be taken into account when calculating the motion plan.
                Defaults to ``None``.
            planner_id (:obj:`str`): Name of the planner to use. Defaults to ``rrtconnect``.
            trials (:obj:`int`): Number of search trials to run. Defaults to ``1``.
            resolution (:obj:`float`): Validity checking resolution. This value
                is specified as a fraction of the space's extent.
                Defaults to ``0.02``.
            gantry_joint_limits (:obj:`list` of `float`): List of 6 floats defining the upper/lower limits of
                gantry joints. Use this if you want to restrict the working area of the path planner.
            arm_joint_limits (:obj:`list` of `float`): List of 12 floats defining the upper/lower limits of
                arm joints. Use this if you want to restrict the working area of the path planner.
            shallow_state_search (:obj:`bool`): True to search only a minimum of
                valid states before searching a path, False to search states intensively.
            optimize_path_length (:obj:`bool`): True to search the path with minimal total length among all `trials`,
                False to return the first valid path found. It only affects the output if `trials > 1`.

        Returns:
            list: List of :class:`Configuration` objects representing the
            collision-free path to the ``goal_frame``.
        """
        assert_robot(robot)
        return self._find_path_plan(robot, {'target_type': 'pose', 'target': goal_frame},
                                    metric_values, collision_meshes, planner_id, trials, resolution,
                                    gantry_joint_limits, arm_joint_limits, shallow_state_search, optimize_path_length)

    def add_building_member(self, robot, building_member_mesh):
        """Adds a building member to the 3D scene and attaches it to the robot.

        Args:
            robot (:class:`Robot`): Robot instance to attach the building member to.
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


def assert_robot(robot):
    if not robot:
        raise ValueError('No instance of robot found')
    if not robot.model:
        raise ValueError('The robot instance has no model information attached')
    if 'index' not in robot.model.attr:
        raise ValueError('Robot model needs to define an index as part of the model.attr dictionary')


# --------------------------------------------------------------------------
# NETWORKING HELPERS
# A couple of simple networking helpers for host name resolution
# --------------------------------------------------------------------------


def is_ipv4_address(addr):
    try:
        socket.inet_aton(addr)
        return True
    except socket.error:
        return False


def resolve_host(host):
    if is_ipv4_address(host):
        return host
    else:
        return socket.gethostbyname(host)


# --------------------------------------------------------------------------
# MAPPINGS
# The following mapping functions are only internal to make sure
# all transformations from and to V-REP are consistent
# --------------------------------------------------------------------------

def vrep_pose_to_frame(pose, scale):
    return Frame.from_list(floats_from_vrep(pose, scale))


def frame_to_vrep_pose(frame, scale):
    # COMPAS FAB uses meters, just like V-REP,
    # so in general, scale should always be 1
    pose = matrix_from_frame(frame)
    pose[0][3] = pose[0][3] / scale
    pose[1][3] = pose[1][3] / scale
    pose[2][3] = pose[2][3] / scale
    return pose[0] + pose[1] + pose[2] + pose[3]


def config_from_vrep(list_of_floats, scale):
    # COMPAS FAB uses radians and meters, just like V-REP,
    # so in general, scale should always be 1
    radians = list_of_floats[3:]
    prismatic_values = map(lambda v: v * scale, list_of_floats[0:3])
    return Configuration.from_prismatic_and_revolute_values(prismatic_values, radians)


def config_to_vrep(config, scale):
    # COMPAS FAB uses radians and meters, just like V-REP,
    # so in general, scale should always be 1
    values = list(map(lambda v: v / scale, config.prismatic_values))
    values.extend(config.revolute_values)
    return values


def floats_to_vrep(list_of_floats, scale):
    return [v / scale for v in list_of_floats]


def floats_from_vrep(list_of_floats, scale):
    return [v * scale for v in list_of_floats]
