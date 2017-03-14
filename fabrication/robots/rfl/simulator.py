from __future__ import print_function

import math
import logging
from timeit import default_timer as timer
from compas.datastructures.mesh import Mesh
from compas_fabrication.fabrication.robots.rfl.vrep_remote_api import vrep
from compas_fabrication.fabrication.robots.rfl import Configuration, Robot

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

    def __init__(self, host='127.0.0.1', port=19997, debug=False):
        self.client_id = None
        self.host = host
        self.port = port
        self.default_timeout_in_ms = -50000000
        self.thread_cycle_in_ms = 5
        self.debug = debug
        self._lua_script_name = 'RFL'
        self._added_handles = []

    def __enter__(self):
        # Stop existing simulation, if any
        vrep.simxFinish(-1)

        if self.debug:
            LOG.debug('Connecting to V-REP...')

        # Connect to V-REP, set a very large timeout for blocking commands
        self.client_id = vrep.simxStart(self.host, self.port, True, True,
                                        self.default_timeout_in_ms,
                                        self.thread_cycle_in_ms)

        # Start simulation
        vrep.simxStartSimulation(self.client_id, DEFAULT_OP_MODE)

        return self

    def __exit__(self, *args):
        self.remove_objects(self._added_handles)

        # Stop simulation
        vrep.simxStopSimulation(self.client_id, DEFAULT_OP_MODE)

        # Close the connection to V-REP
        vrep.simxFinish(self.client_id)
        self.client_id = None

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
            object_name (:obj:`string`): Name of the object.

        Returns:
            int: Object handle.
        """
        _res, handle = vrep.simxGetObjectHandle(self.client_id,
                                                object_name,
                                                DEFAULT_OP_MODE)
        return handle

    def set_metric(self, metric_values):
        """Assigns a metric defining relations between axis values.

        It takes a list of 9 :obj:`float` values (3 for gantry + 6 for joints)
        ranging from 0 to 1, where 1 indicates the axis is blocked and cannot
        move during inverse kinematic solving.

        Args:
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values from 0 to 1.
        """
        vrep.simxCallScriptFunction(self.client_id,
                                    self._lua_script_name,
                                    CHILD_SCRIPT_TYPE, 'setTheMetric',
                                    [], metric_values, [],
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

    def set_robot_config(self, robot, config):
        """Moves the robot the the specified configuration.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            config (:class:`.Configuration`): Instance of robot's
                configuration.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Robot
            >>> with Simulator() as simulator:
            ...     simulator.set_robot_config(Robot(11),
            ...                                Configuration([7.6, -4.5, -4.5],
            ...                                [90, 0, 0, 0, 0, -90]))
            ...
        """
        values = list(config.coordinates)
        values.extend([math.radians(angle) for angle in config.joint_values])

        self.set_metric([0.0] * 9)
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
        return Configuration.from_list(config)

    def find_robot_states(self, robot, goal_pose, metric_values=[0.1] * 9, max_trials=None, max_results=1):
        """Finds valid robot configurations for the specified goal pose.

        Args:
            robot (:class:`.Robot`): Robot instance.
            goal_pose (:obj:`list` of :obj:`float`): Target or goal pose
                specified as a list of 12 :obj:`float` values.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis is blocked and cannot
                move during inverse kinematic solving.
            max_trials (:obj:`int`): Number of trials to run. Set to ``None``
                to retry infinitely.
            max_results (:obj:`int`): Maximum number of result states to return.

        Returns:
            list: List of :class:`Configuration` objects representing the
                collision-free configuration for the ``goal_pose``.
        """
        self.set_metric(metric_values)

        states = self._find_raw_robot_states(robot, goal_pose, max_trials, max_results)

        return [Configuration.from_list(states[i:i + 9])
                for i in range(0, len(states), 9)]

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
            if i > 200 or (res != 0 and not retry_until_success):
                raise SimulationError('Failed to search robot states', res)

            final_states.extend(states)

            if len(final_states) // 9 >= max_results:
                break

        return final_states

    def pick_building_member(self, robot, building_member_mesh, pickup_pose_or_config, metric_values=[0.1] * 9):
        """Picks up a building member and attaches it to the robot.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            building_member_mesh (:class:`compas.datastructures.mesh.Mesh`): Mesh
                of the building member that will be attached to the robot.
            pickup_pose_or_config (:obj:`list` of :obj:`float` or :class:`Configuration` instance):
                Describes the pickup position, either as a pose (list of 12 :obj:`float` values)
                or as a :class:`Configuration`.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis is blocked and cannot
                move during inverse kinematic solving.

        """
        pickup_config = pickup_pose_or_config if isinstance(pickup_pose_or_config, Configuration) else None

        # If we don't have a configuration, then it must be a pose. Try to find a configuration using
        # shallow state search.
        if not pickup_config and pickup_pose_or_config:
            pickup_pose = pickup_pose_or_config
            pickup_config = self.find_robot_states(robot, pickup_pose, metric_values)[-1]

        if pickup_config:
            self.set_robot_config(robot, pickup_config)

        self.add_building_member(robot, building_member_mesh)

    def find_path_plan(self, robot, goal_pose, metric_values=[0.1] * 9, collision_meshes=None,
                       algorithm='rrtconnect', trials=1, resolution=0.02, shallow_state_search=True):
        """Finds a path plan to move the selected robot from its current position
        to the `goal_pose`.

        Args:
            robot (:class:`.Robot`): Robot instance to move.
            goal_pose (:obj:`list` of :obj:`float`): Target or goal pose
                specified as a list of 12 :obj:`float` values.
            metric_values (:obj:`list` of :obj:`float`): 9 :obj:`float`
                values (3 for gantry + 6 for joints) ranging from 0 to 1,
                where 1 indicates the axis is blocked and cannot
                move during inverse kinematic solving.
            collision_meshes (:obj:`list` of :class:`compas.datastructures.mesh.Mesh`): Collision meshes
                to be taken into account when calculating the motion plan.
                Defaults to ``None``.
            algorithm (:obj:`list` of :obj:`float`): 6 joint values
                expressed in degrees. Defaults to ``rrtconnect``.
            trials (:obj:`int`): Number of search trials to run. Defaults to ``1``.
            resolution (:obj:`float`): Validity checking resolution. This value
                is specified as a fraction of the space's extent.
                Defaults to ``0.02``.
            shallow_state_search (:obj:`bool`) True to search only a minimum of
                valid states before searching a path, False to search states intensively.

        Returns:
            list: List of :class:`Configuration` objects representing the
                collision-free path to the ``goal_pose``.
        """
        if algorithm not in self.SUPPORTED_ALGORITHMS:
            raise ValueError('Unsupported algorithm. Must be one of: ' + str(self.SUPPORTED_ALGORITHMS))

        first_start = timer() if self.debug else None
        if collision_meshes:
            self.add_meshes(collision_meshes)
        if self.debug:
            LOG.debug('Execution time: add_meshes=%f.2', timer() - first_start)

        start = timer() if self.debug else None
        self.set_metric(metric_values)
        if self.debug:
            LOG.debug('Execution time: set_metric=%f.2', timer() - start)

        start = timer() if self.debug else None
        max_trials = None if shallow_state_search else 160
        max_results = 1 if shallow_state_search else 80
        states = self._find_raw_robot_states(robot, goal_pose, max_trials, max_results)
        if self.debug:
            LOG.debug('Execution time: search_robot_states=%f.2', timer() - start)

        start = timer() if self.debug else None
        res, _, path, _, _ = self.run_child_script('searchRobotPath',
                                                   [robot.index,
                                                    trials,
                                                    (int)(resolution * 1000)],
                                                   states, [algorithm])
        if self.debug:
            LOG.debug('Execution time: search_robot_path=%f.2', timer() - start)

        if res != 0:
            raise SimulationError('Failed to search robot path', res)

        if self.debug:
            LOG.debug('Execution time: total=%f.2', timer() - first_start)

        return [Configuration.from_list(path[i:i + 9])
                for i in range(0, len(path), 9)]

    def add_building_member(self, robot, building_member_mesh):
        """Add a building member to the RFL scene and attaches it to the robot.

        Args:
            robot (:class:`.Robot`): Robot instance to attach the building member to.
            building_member_mesh (:class:`compas.datastructures.mesh.Mesh`): Mesh
                of the building member that will be attached to the robot.

        Returns:
            int: Object handle (identifier) assigned to the building member.
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
        """
        mesh_handles = []

        for mesh in meshes:
            # TODO: Remove the next two lines and uncomment the third
            # if `to_vertices_and_faces` is merged into compAS.
            vertices = mesh.xyz
            faces = [mesh.face_vertices(fkey, ordered=True) for fkey in mesh.face]
            # vertices, faces = mesh.to_vertices_and_faces()

            vrep_packing = ([item for sublist in vertices for item in sublist] +
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
