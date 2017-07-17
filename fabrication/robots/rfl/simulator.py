from __future__ import print_function

import json
import urllib2
import math
import socket
import logging
from timeit import default_timer as timer
from compas.datastructures.mesh import Mesh
from compas_fabrication.fabrication.robots import Pose
from compas_fabrication.fabrication.robots.rfl import Configuration, Robot
from compas_fabrication.fabrication.robots.rfl.vrep_remote_api import vrep

DEFAULT_SCALE = 1000.
DEFAULT_OP_MODE = vrep.simx_opmode_blocking
CHILD_SCRIPT_TYPE = vrep.sim_scripttype_childscript
LOG = logging.getLogger('compas_fabrication.simulator')


class SimulationError(Exception):
    """Wraps an exception that occurred inside the simulation engine."""

    def __init__(self, message, error_code):
        super(SimulationError, self).__init__('Error code: ' +
                                              str(error_code) +
                                              '; ' + message)
        self.error_code = error_code


class Simulator(object):
    """Interface to run simulations on the RFL using VREP as
    the engine for inverse kinematics.

    :class:`.Simulator` is a context manager type, so it's best used in combination
    with the ``with`` statement to ensure resource deallocation.


    Args:
        host (:obj:`str`): IP address or DNS name of the V-REP simulator.
        port (:obj:`int`): Port of the simulator.
        scale(:obj:`int`): Scaling of the model. Defaults to millimeters (``1000``).
        debug (:obj:`bool`): True to enable debug messages, False otherwise.

    Examples:

        >>> from compas_fabrication.fabrication.robots.rfl import *
        >>> with Simulator() as simulator:
        ...     print ('Connected: %s' % simulator.is_connected())
        ...
        Connected: True

    """
    SUPPORTED_ALGORITHMS = ('bitrrt', 'bkpiece1', 'est', 'kpiece1',
                            'lazyprmstar', 'lbkpiece1', 'lbtrrt', 'pdst',
                            'prm', 'prrt', 'rrt', 'rrtconnect', 'rrtstar',
                            'sbl', 'stride', 'trrt')

    def __init__(self, host='127.0.0.1', port=19997, scale=DEFAULT_SCALE, debug=False):
        self.client_id = None
        self.host = resolve_host(host)
        self.port = port
        self.default_timeout_in_ms = -50000000
        self.thread_cycle_in_ms = 5
        self.debug = debug
        self.scale = float(scale)
        self._lua_script_name = 'RFL'
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

    def is_connected(self):
        """Indicates whether the simulator has an active connection.

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

            >>> from compas_fabrication.fabrication.robots.rfl import Simulator
            >>> with Simulator() as simulator:
            ...     matrices = simulator.get_object_matrices([0])
            ...     print(map(int, matrices[0]))
            [0, 0, 0, -6, 0, 0, 0, -7, 0, 0, 0, -5]

        .. note::
            The resulting dictionary is keyed by object handle.
        """
        _res, _, matrices, _, _ = self.run_child_script('getShapeMatrices', object_handles, [], [])
        return dict([(object_handles[i // 12], floats_from_vrep(matrices[i:i + 12], self.scale)) for i in range(0, len(matrices), 12)])

    def get_all_visible_handles(self):
        """Gets a list of object handles (identifiers) for all visible
        shapes of the RFL model.

        Returns:
            list: List of object handles (identifiers) of the RFL model.
        """
        return self.run_child_script('getRobotVisibleShapeHandles', [], [], [])[1]

    def set_robot_metric(self, robot, metric_values):
        """Assigns a metric defining relations between axis values of a robot.

        It takes a list of 9 :obj:`float` values (3 for gantry + 6 for joints)
        ranging from 0 to 1, where 1 indicates the axis is blocked and cannot
        move during inverse kinematic solving. A value of 1 on any of these
        effectively removes one degree of freedom (DOF).

        Args:
            robot (:class:`.Robot`): Robot instance.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values from 0 to 1.
        """
        vrep.simxCallScriptFunction(self.client_id,
                                    self._lua_script_name,
                                    CHILD_SCRIPT_TYPE, 'setTheMetric',
                                    [robot.index], metric_values, [],
                                    bytearray(), DEFAULT_OP_MODE)

    def reset_all_robots(self):
        """Resets all robots in the RFL to their base configuration.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Simulator
            >>> with Simulator() as simulator:
            ...     simulator.reset_all_robots()
            ...
        """
        for id in Robot.SUPPORTED_ROBOTS:
            Robot(id, client=self).reset_config()

    def set_robot_pose(self, robot, pose):
        """Moves the robot the the specified pose.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            pose (:class:`.Pose`): Target or goal pose instance.

        """
        # First check if the start state is reachable
        config = self.find_robot_states(robot, pose, [0.] * robot.dof)[-1]

        if not config:
            raise ValueError('Cannot find a valid config for the given pose')

        self.set_robot_config(robot, config)

    def set_robot_config(self, robot, config):
        """Moves the robot the the specified configuration.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            config (:class:`Configuration` instance): Describes the position of the
                robot as an instance of :class:`Configuration`.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Robot, Configuration
            >>> with Simulator() as simulator:
            ...     config = Configuration.from_joints_and_external_axes([90, 0, 0, 0, 0, -90],
            ...                                                          [7600, -4500, -4500])
            ...     simulator.set_robot_config(Robot(11), config)
            ...
        """
        if not config:
            raise ValueError('Unsupported config value')

        values = config_to_vrep(config, self.scale)

        self.set_robot_metric(robot, [0.0] * robot.dof)
        self.run_child_script('moveRobotFK',
                              [], values, ['robot' + robot.name])

    def get_robot_config(self, robot):
        """Gets the current configuration of the specified robot.

        Args:
            robot (:class:`.Robot`): Robot instance.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Robot
            >>> with Simulator() as simulator:
            ...     config = simulator.get_robot_config(Robot(11))

        Returns:
            An instance of :class:`.Configuration`.
        """
        res, _, config, _, _ = self.run_child_script('getRobotState',
                                                     [robot.index],
                                                     [], [])
        return config_from_vrep(config, self.scale)

    def find_robot_states(self, robot, goal_pose, metric_values=None, max_trials=None, max_results=1):
        """Finds valid robot configurations for the specified goal pose.

        Args:
            robot (:class:`.Robot`): Robot instance.
            goal_pose (:class:`.Pose`): Target or goal pose instance.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis is blocked and cannot
                move during inverse kinematic solving.
            max_trials (:obj:`int`): Number of trials to run. Set to ``None``
                to retry infinitely.
            max_results (:obj:`int`): Maximum number of result states to return.

        Returns:
            list: List of :class:`Configuration` objects representing
            the collision-free configuration for the ``goal_pose``.
        """
        if not metric_values:
            metric_values = [0.1] * robot.dof

        self.set_robot_metric(robot, metric_values)

        states = self._find_raw_robot_states(robot, pose_to_vrep(goal_pose, self.scale), max_trials, max_results)

        return [config_from_vrep(states[i:i + robot.dof], self.scale)
                for i in range(0, len(states), robot.dof)]

    def _find_raw_robot_states(self, robot, goal_pose, max_trials=None, max_results=1):
        i = 0
        final_states = []
        retry_until_success = True if not max_trials else False

        while True:
            res, _, states, _, _ = self.run_child_script('searchRobotStates',
                                                         [robot.index,
                                                          max_trials or 1,
                                                          max_results],
                                                         goal_pose, [])

            # Even if the retry_until_success is set to True, we short circuit
            # at some point to prevent infinite loops caused by misconfiguration
            i += 1
            if i > 20 or (res != 0 and not retry_until_success):
                raise SimulationError('Failed to search robot states', res)

            final_states.extend(states)

            if len(final_states):
                LOG.info('Found %d valid robot states', len(final_states) // 9)
                break
            else:
                LOG.info('No valid robot states found, will retry.')

        return final_states

    def pick_building_member(self, robot, building_member_mesh, pickup_pose, metric_values=None):
        """Picks up a building member and attaches it to the robot.

        Args:
            robot (:class:`.Robot`): Robot instance to use for pick up.
            building_member_mesh (:class:`compas.datastructures.mesh.Mesh`): Mesh
                of the building member that will be attached to the robot.
            pickup_pose (:class:`.Pose`): Pickup pose instance.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.

        Returns:
            int: Object handle (identifier) assigned to the building member.
        """
        if not metric_values:
            metric_values = [0.1] * robot.dof

        self.set_robot_pose(robot, pickup_pose)

        return self.add_building_member(robot, building_member_mesh)

    def find_path_plan(self, robot, goal_pose, metric_values=None, collision_meshes=None,
                       algorithm='rrtconnect', trials=1, resolution=0.02,
                       gantry_joint_limits=None, arm_joint_limits=None, shallow_state_search=True):
        """Finds a path plan to move the selected robot from its current position
        to the `goal_pose`.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            goal_pose (:class:`.Pose`): Target or goal pose instance.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.
            collision_meshes (:obj:`list` of :class:`compas.datastructures.mesh.Mesh`): Collision meshes
                to be taken into account when calculating the motion plan.
                Defaults to ``None``.
            algorithm (:obj:`str`): Name of the algorithm to use. Defaults to ``rrtconnect``.
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

        Returns:
            list: List of :class:`Configuration` objects representing the
            collision-free path to the ``goal_pose``.
        """
        if not metric_values:
            metric_values = [0.1] * robot.dof

        if algorithm not in self.SUPPORTED_ALGORITHMS:
            raise ValueError('Unsupported algorithm. Must be one of: ' + str(self.SUPPORTED_ALGORITHMS))

        first_start = timer() if self.debug else None
        if collision_meshes:
            self.add_meshes(collision_meshes)
        if self.debug:
            LOG.debug('Execution time: add_meshes=%.2f', timer() - first_start)

        start = timer() if self.debug else None
        self.set_robot_metric(robot, metric_values)
        if self.debug:
            LOG.debug('Execution time: set_robot_metric=%.2f', timer() - start)

        start = timer() if self.debug else None
        max_trials = None if shallow_state_search else 80
        max_results = 1 if shallow_state_search else 80
        states = self._find_raw_robot_states(robot, pose_to_vrep(goal_pose, self.scale), max_trials, max_results)
        if self.debug:
            LOG.debug('Execution time: search_robot_states=%.2f', timer() - start)

        start = timer() if self.debug else None
        string_param_list = [algorithm]
        if gantry_joint_limits or arm_joint_limits:
            joint_limits = []
            joint_limits.extend(floats_to_vrep(gantry_joint_limits or [], self.scale))
            joint_limits.extend(arm_joint_limits or [])
            string_param_list.append(','.join(map(str, joint_limits)))

        if self.debug:
            LOG.debug('About to execute path planner: algorithm=%s, trials=%d, shallow_state_search=%s', algorithm, trials, shallow_state_search)

        res, _, path, _, _ = self.run_child_script('searchRobotPath',
                                                   [robot.index,
                                                    trials,
                                                    (int)(resolution * 1000)],
                                                   states, string_param_list)
        if self.debug:
            LOG.debug('Execution time: search_robot_path=%.2f', timer() - start)

        if res != 0:
            raise SimulationError('Failed to search robot path', res)

        if self.debug:
            LOG.debug('Execution time: total=%.2f', timer() - first_start)

        return [config_from_vrep(path[i:i + robot.dof], self.scale)
                for i in range(0, len(path), robot.dof)]

    def add_building_member(self, robot, building_member_mesh):
        """Adds a building member to the RFL scene and attaches it to the robot.

        Args:
            robot (:class:`.Robot`): Robot instance to attach the building member to.
            building_member_mesh (:class:`compas.datastructures.mesh.Mesh`): Mesh
                of the building member that will be attached to the robot.

        Returns:
            int: Object handle (identifier) assigned to the building member.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        handles = self.add_meshes([building_member_mesh])

        if len(handles) != 1:
            raise SimulationError('Expected one handle, but multiple found=' + str(handles), -1)

        handle = handles[0]

        parent_handle = self.get_object_handle('customGripper' + robot.name + '_connection')
        vrep.simxSetObjectParent(self.client_id, handle, parent_handle, True, DEFAULT_OP_MODE)

        return handle

    def add_meshes(self, meshes):
        """Adds meshes to the RFL scene.

        Args:
            meshes (:obj:`list` of :class:`compas.datastructures.mesh.Mesh`): List
                of meshes to add to the current simulation scene.

        Returns:
            list: List of object handles (identifiers) assigned to the meshes.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        mesh_handles = []

        for mesh in meshes:
            if not mesh.is_trimesh():
                raise ValueError('The simulator only supports tri-meshes')

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
        """Removes meshes from the RFL scene.

        This is functionally identical to ``remove_objects``, but it's here for
        symmetry reasons.

        Args:
            mesh_handles (:obj:`list` of :obj:`int`): Object handles to remove.
        """
        self.remove_objects(mesh_handles)

    def remove_objects(self, object_handles):
        """Removes objects from the RFL scene.

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
                                           self._lua_script_name,
                                           CHILD_SCRIPT_TYPE, function_name,
                                           in_ints, in_floats, in_strings,
                                           bytearray(), DEFAULT_OP_MODE)


class SimulationCoordinator(object):
    """Coordinates the execution of simulation using different strategies.
    For instance, it allows to run a path planning simulation on one node
    or distribute it among many nodes and get multiple solutions as result.

    The coordinator takes as input one large dictionary-like structure with the
    entire definition of a path planning job. The following shows an example of
    this, exposing all possible configuration values::

        {
            'debug': True,
            'trials': 1,
            'shallow_state_search': True,
            'algorithm': 'rrtconnect',
            'resolution': 0.02,
            'collision_meshes': [],
            'robots': [
                {
                    'robot': 12,
                    'start': {
                        'joint_values': [90.0, 100.0, -160.0, 180.0, 30.0, -90.0],
                        'external_axes': [9562.26, -2000, -3600]
                    },
                }
                {
                    'robot': 11,
                    'start': {
                        'joint_values': [90.0, 100.0, -160.0, 180.0, 30.0, -90.0],
                        'external_axes': [9562.26, -1000, -4600]
                    },
                    'goal': {
                        'values': [-0.98, 0.16, 0.0, 1003, 0.0, 0.0, -1.0, -5870, -0.16, -0.98, 0.0, -1500]
                    },
                    'building_member': {
                        'attributes': {
                            'name': 'Mesh',
                        }
                    },
                    'joint_limits': {
                        'gantry': [
                            [0, 20000],
                            [-12000, 0],
                            [-4600, -1000]
                        ],
                        'arm': [
                            [-180, 180],
                            [-90, 150],
                            [-180, 75],
                            [-400, 400],
                            [-125, 120],
                            [-400, 400]
                        ]
                    },
                    'metric_values': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                }
            ]
        }

    """

    @classmethod
    def remote_executor(cls, options, executor_host='127.0.0.1', port=7000):
        url = 'http://%s:%d/path-planner' % (executor_host, port)
        data = json.dumps(options, encoding='ascii')
        request = urllib2.Request(url, data, {'Content-Type': 'application/json', 'Content-Length': str(len(data))})
        f = urllib2.urlopen(request)
        response = f.read()
        f.close()

        # TODO: Do something different here
        # Return the ID of the job and poll
        results = json.loads(response)
        # TODO: Get DOF from robot instance
        return [[config_from_vrep(path_list[i:i + 9], 1)
                for i in range(0, len(path_list), 9)] for path_list in results]

    @classmethod
    def local_executor(cls, options, host='127.0.0.1', port=19997):
        with Simulator(debug=options.get('debug', True), host=host, port=port) as simulator:
            active_robot_options = None

            # Setup all robots' start state
            for r in options['robots']:
                robot = Robot(r['robot'], simulator)

                if 'start' in r:
                    if r['start'].get('joint_values'):
                        start = Configuration.from_data(r['start'])
                    elif r['start'].get('values'):
                        start = Pose.from_data(r['start'])
                        try:
                            reachable_state = simulator.find_robot_states(robot, start, [0.] * robot.dof, 1, 1)
                            start = reachable_state[-1]
                            LOG.info('Robot state found for start pose. External axes=%s, Joint values=%s', str(start.external_axes), str(start.joint_values))
                        except SimulationError:
                            raise ValueError('Start plane is not reachable: %s' % str(r['start']))

                    simulator.set_robot_config(robot, start)

                if 'building_member' in r:
                    simulator.add_building_member(robot, Mesh.from_data(r['building_member']))

                if 'goal' in r:
                    active_robot_options = r

            # Set global scene options
            if 'collision_meshes' in options:
                simulator.add_meshes(map(Mesh.from_data, options['collision_meshes']))

            # Check if there's at least one active robot (i.e. one with a goal defined)
            if active_robot_options:
                robot = Robot(active_robot_options['robot'], simulator)
                if active_robot_options['goal'].get('values'):
                    goal = Pose.from_data(active_robot_options['goal'])
                else:
                    raise ValueError('Unsupported goal type: %s' % str(active_robot_options['goal']))

                kwargs = {}
                kwargs['metric_values'] = active_robot_options.get('metric_values')
                kwargs['algorithm'] = options.get('algorithm')
                kwargs['resolution'] = options.get('resolution')

                if 'joint_limits' in active_robot_options:
                    joint_limits = active_robot_options['joint_limits']
                    if joint_limits.get('gantry'):
                        kwargs['gantry_joint_limits'] = [item for sublist in joint_limits.get('gantry') for item in sublist]
                    if joint_limits.get('arm'):
                        kwargs['arm_joint_limits'] = [item for sublist in joint_limits.get('arm') for item in sublist]

                kwargs['trials'] = options.get('trials')
                kwargs['shallow_state_search'] = options.get('shallow_state_search')

                # Filter None values
                kwargs = {k: v for k, v in kwargs.iteritems() if v is not None}

                path = simulator.find_path_plan(robot, goal, **kwargs)
                LOG.info('Found path of %d steps', len(path))
            else:
                robot = Robot(options['robots'][0]['robot'], simulator)
                path = [simulator.get_robot_config(robot)]

        return path


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

def pose_to_vrep(pose, scale):
    pose = list(pose.values)
    pose[3] = pose[3] / scale
    pose[7] = pose[7] / scale
    pose[11] = pose[11] / scale
    return pose


def config_from_vrep(list_of_floats, scale):
    angles = map(math.degrees, list_of_floats[3:])
    external_axes = map(lambda v: v * scale, list_of_floats[0:3])
    return Configuration.from_joints_and_external_axes(angles, external_axes)


def config_to_vrep(config, scale):
    values = map(lambda v: v / scale, config.external_axes)
    values.extend([math.radians(angle) for angle in config.joint_values])
    return values


def floats_to_vrep(list_of_floats, scale):
    return [v / scale for v in list_of_floats]


def floats_from_vrep(list_of_floats, scale):
    return [v * scale for v in list_of_floats]
