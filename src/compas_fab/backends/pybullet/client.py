from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import shutil
import tempfile
from itertools import combinations

import compas
from compas._os import system
from compas.geometry import Frame
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
        if self.connection_type == 'gui' and system != 'darwin' and not compas.is_windows() and ('DISPLAY' not in os.environ):
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

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def step_simulation(self):
        """By default, the physics server will not step the simulation,
        unless you explicitly send a ``step_simulation`` command.  This
        method will perform all the actions in a single forward dynamics
        simulation step such as collision detection, constraint solving
        and integration. The timestep is 1/240 second.
        """
        pybullet.stepSimulation(physicsClientId=self.client_id)

    def load_robot(self, urdf_file):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_file : :obj:`str` or file object
            Absolute file path to the urdf file name or file object. The mesh file can be linked by either
            `"package::"` or relative path.

        Notes
        -----
        By default, PyBullet will use the convex hull of any mesh loaded from a URDF for collision detection.
        Amending the link tag as ``<link concave="yes" name="<name of link>">`` will make the mesh concave
        for static meshes (see this `example <https://github.com/bulletphysics/bullet3/blob/master/data/samurai.urdf>`_).
        For non-static concave meshes, replace the OBJ files within the URDF with those generated by ``pybullet.vhacd``.
        """
        robot_model = RobotModel.from_urdf_file(urdf_file)
        robot = Robot(robot_model, client=self)

        with redirect_stdout(enabled=not self.verbose):
            pybullet_uid = pybullet.loadURDF(urdf_file, useFixedBase=True,
                                             physicsClientId=self.client_id,
                                             flags=pybullet.URDF_USE_SELF_COLLISION)
            robot.attributes['pybullet_uid'] = pybullet_uid

        self._add_ids_to_robot_joints(robot)
        self._add_ids_to_robot_links(robot)
        return robot

    def _add_ids_to_robot_joints(self, robot):
        joint_ids = self._get_joint_ids(robot.attributes['pybullet_uid'])
        for joint_id in joint_ids:
            joint_name = self._get_joint_name(joint_id, robot.attributes['pybullet_uid'])
            joint = robot.model.get_joint_by_name(joint_name)
            pybullet_attr = {'id': joint_id}
            joint.attr.setdefault('pybullet', {}).update(pybullet_attr)

    def _add_ids_to_robot_links(self, robot):
        joint_ids = self._get_joint_ids(robot.attributes['pybullet_uid'])
        for link_id in joint_ids:
            link_name = self._get_link_name(link_id, robot.attributes['pybullet_uid'])
            link = robot.model.get_link_by_name(link_name)
            pybullet_attr = {'id': link_id}
            link.attr.setdefault('pybullet', {}).update(pybullet_attr)

    def _get_joint_id_by_name(self, name, robot):
        return robot.model.get_joint_by_name(name).attr['pybullet']['id']

    def _get_joint_ids_by_name(self, names, robot):
        return tuple(self._get_joint_id_by_name(name, robot) for name in names)

    def _get_link_id_by_name(self, name, robot):
        return robot.model.get_link_by_name(name).attr['pybullet']['id']

    def _get_link_ids_by_name(self, names, robot):
        return tuple(self._get_link_id_by_name(name, robot) for name in names)

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
        if configuration:
            joint_ids = self._get_joint_ids_by_name(configuration.joint_names, robot)
            self._set_joint_positions(joint_ids, configuration.values, robot.attributes['pybullet_uid'])
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
                self._check_collision(robot.attributes['pybullet_uid'], 'robot', body_id, name)

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
        link_names = robot.get_link_names_with_collision_geometry()
        # check for collisions between robot links
        for link_1_name, link_2_name in combinations(link_names, 2):
            if {link_1_name, link_2_name} in self.disabled_collisions:
                continue
            link_1_id = self._get_link_id_by_name(link_1_name, robot)
            link_2_id = self._get_link_id_by_name(link_2_name, robot)
            self._check_collision(robot.attributes['pybullet_uid'], link_1_name, robot.attributes['pybullet_uid'], link_2_name, link_1_id, link_2_id)
        # check for collisions between robot links and attached collision objects
        for link_name in link_names:
            link_id = self._get_link_id_by_name(link_name, robot)
            for name, constraint_info_list in self.attached_collision_objects.items():
                for constraint_info in constraint_info_list:
                    self._check_collision(robot.attributes['pybullet_uid'], link_name, constraint_info.body_id, name, link_id)

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
        default_config = robot.zero_configuration()
        full_configuration = robot.merge_group_with_full_configuration(configuration, default_config, group)
        joint_ids = self._get_joint_ids_by_name(full_configuration.joint_names, robot)
        self._set_joint_positions(joint_ids, full_configuration.values, robot.attributes['pybullet_uid'])
        return full_configuration

    # =======================================
    def convert_mesh_to_body(self, mesh, frame, _name=None, concavity=False, mass=const.STATIC_MASS):
        """Convert compas mesh and its frame to a pybullet body

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
        """
        tmp_dir = tempfile.mkdtemp()
        tmp_obj_path = os.path.join(tmp_dir, 'temp.obj')
        try:
            mesh.to_obj(tmp_obj_path)
            tmp_obj_path = self._handle_concavity(tmp_obj_path, tmp_dir, concavity, mass)
            pyb_body_id = self.body_from_obj(tmp_obj_path, concavity=concavity, mass=mass)
            self._set_base_frame(frame, pyb_body_id)
            # The following lines are for visual debugging purposes
            # To be deleted or rewritten later.
            # if name:
            #     from pybullet_planning import add_body_name
            #     add_body_name(pyb_body, name)
        finally:
            shutil.rmtree(tmp_dir)
        return pyb_body_id

    @staticmethod
    def _handle_concavity(tmp_obj_path, tmp_dir, concavity, mass):
        if not concavity or mass == const.STATIC_MASS:
            return tmp_obj_path
        tmp_vhacd_obj_path = os.path.join(tmp_dir, 'vhacd_temp.obj')
        tmp_log_path = os.path.join(tmp_dir, 'log.txt')
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
