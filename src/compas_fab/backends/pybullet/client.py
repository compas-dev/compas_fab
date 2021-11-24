from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools
import os
import tempfile
from itertools import combinations

import compas
from compas.files import URDF
from compas.geometry import Frame
from compas.robots import MeshDescriptor
from compas.robots import RobotModel

from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab.robots import Robot
from compas_fab.utilities import LazyLoader

from . import const
from .conversions import frame_from_pose
from .conversions import pose_from_frame
from .exceptions import CollisionError
from .planner import PyBulletPlanner
from .utils import LOG
from .utils import redirect_stdout

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletClient',
]


class PyBulletBase(object):
    def __init__(self, connection_type):
        self.client_id = None
        self.connection_type = connection_type

    def connect(self, shadows=True, color=None, width=None, height=None):
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
        self._detect_display()
        options = self._compose_options(color, width, height)
        with redirect_stdout():
            self.client_id = pybullet.connect(const.CONNECTION_TYPE[self.connection_type], options=options)
        if self.client_id < 0:
            raise Exception('Error in establishing connection with PyBullet.')
        if self.connection_type == 'gui':
            self._configure_debug_visualizer(shadows)

    def _detect_display(self):
        if self.connection_type == 'gui' and not compas.OSX and not compas.WINDOWS and ('DISPLAY' not in os.environ):
            self.connection_type = 'direct'
            print('No display detected! Continuing without GUI.')

    @staticmethod
    def _compose_options(color, width, height):
        options = ''
        if color is not None:
            options += '--background_color_red={} --background_color_green={} --background_color_blue={}'.format(*color)
        if width is not None:
            options += '--width={}'.format(width)
        if height is not None:
            options += '--height={}'.format(height)
        return options

    def _configure_debug_visualizer(self, shadows):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=self.client_id)
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
        return pybullet.getConnectionInfo(physicsClientId=self.client_id)['isConnected'] == 1


class PyBulletClient(PyBulletBase, ClientInterface):
    """Interface to use pybullet as backend.

    :class:`compasfab.backends.PyBulletClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

    Thanks to Yijiang Huang and his work in `pybullet_planning
    <https://github.com/yijiangh/pybullet_planning>`_ for much inspiration.

    Parameters
    ----------
    connection_type : :obj:`str`
        Sets the connection type. Defaults to ``'gui'``.
    verbose : :obj:`bool`
        Use verbose logging. Defaults to ``False``.

    Examples
    --------
    >>> from compas_fab.backends import PyBulletClient
    >>> with PyBulletClient(connection_type='direct') as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: True

    """
    def __init__(self, connection_type='gui', verbose=False):
        super(PyBulletClient, self).__init__(connection_type)
        self.planner = PyBulletPlanner(self)
        self.verbose = verbose
        self.collision_objects = {}
        self.attached_collision_objects = {}
        self.disabled_collisions = set()
        self._cache_dir = None

    def __enter__(self):
        self._cache_dir = tempfile.TemporaryDirectory()
        self.connect()
        return self

    def __exit__(self, *args):
        self._cache_dir.cleanup()
        self.disconnect()

    @property
    def unordered_disabled_collisions(self):
        return {frozenset(pair) for pair in self.disabled_collisions}

    def step_simulation(self):
        """By default, the physics server will not step the simulation,
        unless you explicitly send a ``step_simulation`` command.  This
        method will perform all the actions in a single forward dynamics
        simulation step such as collision detection, constraint solving
        and integration. The timestep is 1/240 second.
        """
        pybullet.stepSimulation(physicsClientId=self.client_id)

    def load_robot(self, urdf_file, resource_loaders=None, concavity=False):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_file : :obj:`str` or file object
            Absolute file path to the urdf file name or file object. The mesh file can be linked by either
            `"package::"` or relative path.
        resource_loaders : :obj:`list`
            List of :class:`compas.robots.AbstractMeshLoader` for loading geometry of the robot.  That the
            geometry of the robot model is loaded is required before adding or removing attached collision meshes
            to or from the scene. Defaults to the empty list.
        concavity : :obj:`bool`
            When ``False`` (the default), the mesh will be loaded as its
            convex hull for collision checking purposes.  When ``True``,
            a non-static mesh will be decomposed into convex parts using v-HACD.

        Notes
        -----
        By default, PyBullet will use the convex hull of any mesh loaded from a URDF for collision detection.
        Amending the link tag as ``<link concave="yes" name="<name of link>">`` will make the mesh concave
        for static meshes (see this `example <https://github.com/bulletphysics/bullet3/blob/master/data/samurai.urdf>`_).
        For non-static concave meshes, use the ``concavity`` flag.
        """
        robot_model = RobotModel.from_urdf_file(urdf_file)
        robot = Robot(robot_model, client=self)
        robot.attributes['pybullet'] = {}
        if resource_loaders:
            robot_model.load_geometry(*resource_loaders)
            self.cache_robot(robot, concavity)
        else:
            robot.attributes['pybullet']['cached_robot'] = robot.model
            robot.attributes['pybullet']['cached_robot_filepath'] = urdf_file

        urdf_fp = robot.attributes['pybullet']['cached_robot_filepath']

        self._load_robot_to_pybullet(urdf_fp, robot)

        return robot

    def _load_robot_to_pybullet(self, urdf_file, robot):
        cached_robot = self.get_cached_robot(robot)
        with redirect_stdout(enabled=not self.verbose):
            pybullet_uid = pybullet.loadURDF(urdf_file, useFixedBase=True,
                                             physicsClientId=self.client_id,
                                             flags=pybullet.URDF_USE_SELF_COLLISION)
            cached_robot.attr['uid'] = pybullet_uid

        self._add_ids_to_robot_joints(cached_robot)
        self._add_ids_to_robot_links(cached_robot)

    def reload_from_cache(self, robot):
        """Reloads the PyBullet server with the robot's cached model.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot to be saved for use with PyBullet.

        """
        current_configuration = self.get_robot_configuration(robot)
        cached_robot_model = self.get_cached_robot(robot)
        cached_robot_filepath = self.get_cached_robot_filepath(robot)
        robot_uid = self.get_uid(cached_robot_model)
        pybullet.removeBody(robot_uid, physicsClientId=self.client_id)

        cached_robot_model.to_urdf_file(cached_robot_filepath, prettify=True)
        pybullet.setPhysicsEngineParameter(enableFileCaching=0)
        self._load_robot_to_pybullet(cached_robot_filepath, robot)
        pybullet.setPhysicsEngineParameter(enableFileCaching=1)

        self.set_robot_configuration(robot, current_configuration)
        self.step_simulation()

    def cache_robot(self, robot, concavity=False):
        """Saves an editable copy of the robot's model and its meshes
        for shadowing the state of the robot on the PyBullet server.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot to be saved for use with PyBullet.
        concavity : :obj:`bool`
            When ``False`` (the default), the mesh will be loaded as its
            convex hull for collision checking purposes.  When ``True``,
            a non-static mesh will be decomposed into convex parts using v-HACD.

        Raises
        ------
        :exc:`Exception`
            If geometry has not been loaded.

        """
        robot.ensure_geometry()
        # write meshes to cache
        address_dict = {}
        # must work with given robot.model here because it has the geometry loaded
        for link in robot.model.links:
            for element in itertools.chain(link.visual, link.collision):
                shape = element.geometry.shape
                if isinstance(shape, MeshDescriptor):
                    mesh = shape.geometry
                    mesh_file_name = str(mesh.guid) + '.obj'
                    fp = os.path.join(self._cache_dir.name, mesh_file_name)
                    mesh.to_obj(fp)
                    fp = self._handle_concavity(fp, self._cache_dir.name, concavity, 1, str(mesh.guid))
                    address_dict[shape.filename] = fp

        # create urdf with new mesh locations
        urdf = URDF.from_robot(robot.model)
        meshes = list(urdf.xml.root.iter('mesh'))
        for mesh in meshes:
            filename = mesh.attrib['filename']
            mesh.attrib['filename'] = address_dict[filename]

        # write urdf
        cached_robot_file_name = str(robot.model.guid) + '.urdf'
        cached_robot_filepath = os.path.join(self._cache_dir.name, cached_robot_file_name)
        urdf.to_file(cached_robot_filepath, prettify=True)
        cached_robot = RobotModel.from_urdf_file(cached_robot_filepath)
        robot.attributes['pybullet']['cached_robot'] = cached_robot
        robot.attributes['pybullet']['cached_robot_filepath'] = cached_robot_filepath
        robot.attributes['pybullet']['robot_geometry_cached'] = True

    @staticmethod
    def ensure_cached_robot(robot):
        """Checks if a :class:`compas_fab.robots.Robot` has been cached for use with PyBullet."""
        if not robot.attributes['pybullet']['cached_robot']:
            raise Exception(
                'This method is only callable once the robot has been cached.')

    @staticmethod
    def ensure_cached_robot_geometry(robot):
        """Checks if the geometry of a :class:`compas_fab.robots.Robot` has been cached for use with PyBullet."""
        if not robot.attributes['pybullet'].get('robot_geometry_cached'):
            raise Exception(
                'This method is only callable once the robot with loaded geometry has been cached.')

    def get_cached_robot(self, robot):
        """Returns the editable copy of the robot's model for shadowing the state
        of the robot on the PyBullet server.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot saved for use with PyBullet.

        Returns
        -------
        :class:`compas.robots.RobotModel`

        Raises
        ------
        :exc:`Exception`
            If the robot has not been cached.

        """
        self.ensure_cached_robot(robot)
        return robot.attributes['pybullet']['cached_robot']

    def get_cached_robot_filepath(self, robot):
        """Returns the filepath of the editable copy of the robot's model for shadowing the state
        of the robot on the PyBullet server.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot saved for use with PyBullet.

        Returns
        -------
        :obj:`str`

        Raises
        ------
        :exc:`Exception`
            If the robot has not been cached.

        """
        self.ensure_cached_robot(robot)
        return robot.attributes['pybullet']['cached_robot_filepath']

    def get_uid(self, cached_robot):
        """Returns the internal PyBullet id of the robot's model for shadowing the state
        of the robot on the PyBullet server.

        Parameters
        ----------
        cached_robot : :class:`compas.robots.RobotModel`
            The robot model saved for use with PyBullet.

        Returns
        -------
        :obj:`int`

        """
        return cached_robot.attr['uid']

    def _add_ids_to_robot_joints(self, cached_robot):
        body_id = self.get_uid(cached_robot)
        joint_ids = self._get_joint_ids(body_id)
        for joint_id in joint_ids:
            joint_name = self._get_joint_name(joint_id, body_id)
            joint = cached_robot.get_joint_by_name(joint_name)
            pybullet_attr = {'id': joint_id}
            joint.attr.setdefault('pybullet', {}).update(pybullet_attr)

    def _add_ids_to_robot_links(self, cached_robot):
        body_id = self.get_uid(cached_robot)
        joint_ids = self._get_joint_ids(body_id)
        for link_id in joint_ids:
            link_name = self._get_link_name(link_id, body_id)
            link = cached_robot.get_link_by_name(link_name)
            pybullet_attr = {'id': link_id}
            link.attr.setdefault('pybullet', {}).update(pybullet_attr)

    def _get_joint_id_by_name(self, name, cached_robot):
        return cached_robot.get_joint_by_name(name).attr['pybullet']['id']

    def _get_joint_ids_by_name(self, names, cached_robot):
        return tuple(self._get_joint_id_by_name(name, cached_robot) for name in names)

    def _get_link_id_by_name(self, name, cached_robot):
        return cached_robot.get_link_by_name(name).attr['pybullet']['id']

    def _get_link_ids_by_name(self, names, cached_robot):
        return tuple(self._get_link_id_by_name(name, cached_robot) for name in names)

    def filter_configurations_in_collision(self, robot, configurations):
        """Filters from a list of configurations those which are in collision.
        Used for a custom inverse kinematics function.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configurations may be in collision.
        configurations : :obj:`list` of :class:`compas_fab.robots.Configuration`
            List of configurations to be checked for collisions.

        Returns
        -------
        :obj:`list` of :class:`compas_fab.robots.Configuration`
            The same list of configurations with those in collision replaced with ``None``.
        """
        for i, configuration in enumerate(configurations):
            if not configuration:  # if an ik solution was already removed
                continue
            try:
                self.check_collisions(robot, configuration)
            except CollisionError:
                configurations[i] = None

    # =======================================
    def check_collisions(self, robot, configuration=None):
        """Checks whether the current or given configuration is in collision.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configuration may be in collision.
        configuration : :class:`compas_fab.robots.Configuration`
            Configuration to be checked for collisions.  If ``None`` is given, the current
            configuration will be checked.  Defaults to ``None``.

        Raises
        -------
        :class:`compas_fab.backends.pybullet.DetectedCollision`
        """
        cached_robot = self.get_cached_robot(robot)
        body_id = self.get_uid(cached_robot)
        if configuration:
            joint_ids = self._get_joint_ids_by_name(configuration.joint_names, cached_robot)
            self._set_joint_positions(joint_ids, configuration.joint_values, body_id)
        self.check_collision_with_objects(robot)
        self.check_robot_self_collision(robot)

    def check_collision_with_objects(self, robot):
        """Checks whether the robot and its attached collision objects with its current
        configuration is is colliding with any collision objects.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configuration may be in collision.

        Raises
        -------
        :class:`compas_fab.backends.pybullet.DetectedCollision`
        """
        for name, body_ids in self.collision_objects.items():
            for body_id in body_ids:
                self._check_collision(self.get_uid(self.get_cached_robot(robot)), 'robot', body_id, name)

    def check_robot_self_collision(self, robot):
        """Checks whether the robot and its attached collision objects with its current
        configuration is colliding with itself.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Robot whose configuration may be in collision.

        Raises
        -------
        :class:`compas_fab.backends.pybullet.DetectedCollision`
        """
        cached_robot = self.get_cached_robot(robot)
        body_id = self.get_uid(cached_robot)
        link_names = [link.name for link in cached_robot.iter_links() if link.collision]
        # check for collisions between robot links
        for link_1_name, link_2_name in combinations(link_names, 2):
            if {link_1_name, link_2_name} in self.unordered_disabled_collisions:
                continue
            link_1_id = self._get_link_id_by_name(link_1_name, cached_robot)
            link_2_id = self._get_link_id_by_name(link_2_name, cached_robot)
            self._check_collision(body_id, link_1_name, body_id, link_2_name, link_1_id, link_2_id)

    def check_collision_objects_for_collision(self):
        """Checks whether any of the collision objects are colliding.

        Raises
        -------
        :class:`compas_fab.backends.CollisionError`
        """
        names = self.collision_objects.keys()
        for name_1, name_2 in combinations(names, 2):
            for body_1_id in self.collision_objects[name_1]:
                for body_2_id in self.collision_objects[name_2]:
                    self._check_collision(body_1_id, name_1, body_2_id, name_2)

    def _check_collision(self, body_1_id, body_1_name, body_2_id, body_2_name, link_index_1=None, link_index_2=None):
        kwargs = {
            'bodyA': body_1_id,
            'bodyB': body_2_id,
            'distance': 0,
            'physicsClientId': self.client_id,
            'linkIndexA': link_index_1,
            'linkIndexB': link_index_2,
        }
        kwargs = {key: value for key, value in kwargs.items() if value is not None}
        pts = pybullet.getClosestPoints(**kwargs)
        if pts:
            LOG.warning("Collision between '{}' and '{}'".format(body_1_name, body_2_name))
            raise CollisionError(body_1_name, body_2_name)

    # ======================================
    def _get_base_frame(self, body_id):
        pose = pybullet.getBasePositionAndOrientation(body_id, physicsClientId=self.client_id)
        return frame_from_pose(pose)

    def _get_base_name(self, body_id):
        return self._get_body_info(body_id).base_name.decode(encoding='UTF-8')

    def _get_link_state(self, link_id, body_id):
        return const.LinkState(*pybullet.getLinkState(body_id, link_id, computeForwardKinematics=True, physicsClientId=self.client_id))

    def _get_joint_state(self, joint_id, body_id):
        return const.JointState(*pybullet.getJointState(body_id, joint_id, physicsClientId=self.client_id))

    def _get_joint_states(self, joint_ids, body_id):
        return [const.JointState(*js) for js in pybullet.getJointStates(body_id, joint_ids, physicsClientId=self.client_id)]

    def _get_body_info(self, body_id):
        return const.BodyInfo(*pybullet.getBodyInfo(body_id, physicsClientId=self.client_id))

    def _get_joint_info(self, joint_id, body_id):
        return const.JointInfo(*pybullet.getJointInfo(body_id, joint_id, physicsClientId=self.client_id))

    def _get_num_joints(self, body_id):
        return pybullet.getNumJoints(body_id, physicsClientId=self.client_id)

    def _get_joint_ids(self, body_id):
        return list(range(self._get_num_joints(body_id)))

    def _get_joint_name(self, joint_id, body_id):
        return self._get_joint_info(joint_id, body_id).jointName.decode('UTF-8')

    def _get_link_name(self, link_id, body_id):
        if link_id == const.BASE_LINK_ID:
            return self._get_base_name(body_id)
        return self._get_joint_info(link_id, body_id).linkName.decode('UTF-8')

    def _get_link_frame(self, link_id, body_id):
        if link_id == const.BASE_LINK_ID:
            return self._get_base_frame(body_id)
        link_state = self._get_link_state(link_id, body_id)
        pose = (link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation)
        return frame_from_pose(pose)

    # =======================================
    def _set_base_frame(self, frame, body_id):
        point, quaternion = pose_from_frame(frame)
        pybullet.resetBasePositionAndOrientation(body_id, point, quaternion, physicsClientId=self.client_id)

    def _set_joint_position(self, joint_id, value, body_id):
        pybullet.resetJointState(body_id, joint_id, value, targetVelocity=0, physicsClientId=self.client_id)

    def _set_joint_positions(self, joint_ids, values, body_id):
        if len(joint_ids) != len(values):
            raise Exception('Joints and values must have the same length.')
        for joint_id, value in zip(joint_ids, values):
            self._set_joint_position(joint_id, value, body_id)

    def _get_joint_positions(self, joint_ids, body_id):
        joint_states = self._get_joint_states(joint_ids, body_id)
        return [js.jointPosition for js in joint_states]

    def set_robot_configuration(self, robot, configuration, group=None):
        """Sets the robot's pose to the given configuration. Should be followed by
        `step_simulation` for visualization purposes.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`  The robot to be configured.
        configuration : :class:`compas_fab.robots.Configuration`  If a full configuration is not given,
            the values from :meth:`compas_fab.robots.Robot.zero_configuration` will be used for the
            missing ones.  Joint names are expected to be supplied in the configuration.
        group : :obj:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        """
        cached_robot = self.get_cached_robot(robot)
        body_id = self.get_uid(cached_robot)
        default_config = robot.zero_configuration()
        full_configuration = robot.merge_group_with_full_configuration(configuration, default_config, group)
        joint_ids = self._get_joint_ids_by_name(full_configuration.joint_names, cached_robot)
        self._set_joint_positions(joint_ids, full_configuration.joint_values, body_id)
        return full_configuration

    def get_robot_configuration(self, robot):
        """Gets the robot's current pose.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`  The robot to be configured.

        Returns
        -------
        :class:`compas.robots.Configuration`
        """
        cached_robot = self.get_cached_robot(robot)
        body_id = self.get_uid(cached_robot)
        default_config = robot.zero_configuration()
        joint_ids = self._get_joint_ids_by_name(default_config.joint_names, cached_robot)
        joint_values = self._get_joint_positions(joint_ids, body_id)
        default_config.joint_values = joint_values
        return default_config

    # =======================================
    def convert_mesh_to_body(self, mesh, frame, _name=None, concavity=False, mass=const.STATIC_MASS):
        """Convert compas mesh and its frame to a pybullet body.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
        frame : :class:`compas.geometry.Frame`
        _name : :obj:`str`, optional
            Name of the mesh for tagging in PyBullet's GUI
        concavity : :obj:`bool`, optional
            When ``False`` (the default), the mesh will be loaded as its convex hull for collision checking purposes.
            When ``True``, a non-static mesh will be decomposed into convex parts using v-HACD.
        mass : :obj:`float`, optional
            Mass of the body to be created, in kg.  If ``0`` mass is given (the default),
            the object is static.

        Returns
        -------
        :obj:`int`

        Notes
        -----
        If this method is called several times with the same ``mesh`` instance, but the ``mesh`` has been modified
        in between calls, PyBullet's default caching behavior will prevent it from recognizing these changes.  It
        is best practice to create a new mesh instance or to make use of the `frame` argument, if applicable.  If
        this is not possible, PyBullet's caching behavior can be changed with
        ``pybullet.setPhysicsEngineParameter(enableFileCaching=0)``.
        """
        tmp_obj_path = os.path.join(self._cache_dir.name, '{}.obj'.format(mesh.guid))
        mesh.to_obj(tmp_obj_path)
        tmp_obj_path = self._handle_concavity(tmp_obj_path, self._cache_dir.name, concavity, mass)
        pyb_body_id = self.body_from_obj(tmp_obj_path, concavity=concavity, mass=mass)
        self._set_base_frame(frame, pyb_body_id)
        # The following lines are for visual debugging purposes
        # To be deleted or rewritten later.
        # if name:
        #     from pybullet_planning import add_body_name
        #     add_body_name(pyb_body, name)
        return pyb_body_id

    @staticmethod
    def _handle_concavity(tmp_obj_path, tmp_dir, concavity, mass, mesh_name=''):
        if not concavity or mass == const.STATIC_MASS:
            return tmp_obj_path
        if mesh_name:
            mesh_name += '_'
        tmp_vhacd_obj_path = os.path.join(tmp_dir, mesh_name + 'vhacd_temp.obj')
        tmp_log_path = os.path.join(tmp_dir, mesh_name + 'log.txt')
        with redirect_stdout():
            pybullet.vhacd(tmp_obj_path, tmp_vhacd_obj_path, tmp_log_path)
        return tmp_vhacd_obj_path

    def body_from_obj(self, path, scale=1., concavity=False, mass=const.STATIC_MASS, collision=True, color=const.GREY):
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
        return pybullet.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id, physicsClientId=self.client_id)

    @staticmethod
    def _create_collision_shape(collision_args):
        return pybullet.createCollisionShape(**collision_args)

    @staticmethod
    def _create_visual_shape(visual_args):
        if visual_args.get('rgbaColor') is None:
            return const.NULL_ID
        return pybullet.createVisualShape(**visual_args)

    def _get_visual_args(self, geometry_args, frame=Frame.worldXY(), color=const.RED, specular=None):
        point, quaternion = pose_from_frame(frame)
        visual_args = {
            'rgbaColor': color,
            'visualFramePosition': point,
            'visualFrameOrientation': quaternion,
            'physicsClientId': self.client_id,
        }
        visual_args.update(geometry_args)
        if specular is not None:
            visual_args['specularColor'] = specular
        return visual_args

    def _get_collision_args(self, geometry_args, frame=Frame.worldXY()):
        point, quaternion = pose_from_frame(frame)
        collision_args = {
            'collisionFramePosition': point,
            'collisionFrameOrientation': quaternion,
            'physicsClientId': self.client_id,
        }
        collision_args.update(geometry_args)
        if 'length' in collision_args:
            # pybullet bug visual => length, collision => height
            collision_args['height'] = collision_args['length']
            del collision_args['length']
        return collision_args

    @staticmethod
    def _get_geometry_args(path, concavity=False, scale=1.):
        geometry_args = {
            'shapeType': pybullet.GEOM_MESH,
            'fileName': path,
            'meshScale': [scale] * 3,
        }
        if concavity:
            geometry_args['flags'] = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
        return geometry_args
