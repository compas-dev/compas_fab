from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import tempfile
from itertools import combinations

import compas
from compas._os import system
from compas.geometry import Frame
from compas.robots import RobotModel

from compas_fab.backends.interfaces.client import ClientInterface
from . import const as const, DetectedCollision
from compas_fab.robots import Robot
from compas_fab.utilities import LazyLoader

from .planner import PyBulletPlanner
from .utils import LOG
from .utils import redirect_stdout
from .conversions import frame_from_pose
from .conversions import pose_from_frame

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletClient',
]


class PyBulletBase(object):
    def __init__(self, use_gui):
        self.client_id = None
        self.use_gui = use_gui

    def connect(self, shadows=True, color=None, width=None, height=None):
        # Shared Memory: execute the physics simulation and rendering in a separate process
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7
        self.detect_display()
        method = pybullet.GUI if self.use_gui else pybullet.DIRECT
        options = self.compose_options(color, width, height)
        with redirect_stdout():
            self.client_id = pybullet.connect(method, options=options)
        if self.client_id < 0:
            raise Exception('Error in establishing connection with PyBullet.')
        if self.use_gui:
            self.configure_debug_visualizer(shadows)

    def detect_display(self):
        if self.use_gui and system != 'darwin' and not compas.is_windows() and ('DISPLAY' not in os.environ):
            self.use_gui = False
            print('No display detected! Continuing without GUI.')

    @staticmethod
    def compose_options(color, width, height):
        options = ''
        if color is not None:
            options += '--background_color_red={} --background_color_green={} --background_color_blue={}'.format(*color)
        if width is not None:
            options += '--width={}'.format(width)
        if height is not None:
            options += '--height={}'.format(height)
        return options

    def configure_debug_visualizer(self, shadows):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=self.client_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, shadows, physicsClientId=self.client_id)

    def disconnect(self):
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
    use_gui : :obj:`bool`
        Enable pybullet GUI. Defaults to ``True``.
    verbose : :obj:`bool`
        Use verbose logging. Defaults to ``False``.

    Examples
    --------
    >>> from compas_fab.backends import PyBulletClient
    >>> with PyBulletClient(use_gui=False) as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: True

    """
    def __init__(self, use_gui=True, verbose=False):
        super(PyBulletClient, self).__init__(use_gui)
        self.planner = PyBulletPlanner(self)
        self.verbose = verbose
        self._robot = None
        self.robot_uid = None
        self.collision_objects = {}
        self.attached_collision_objects = {}
        self.collision_map = {}
        self.joint_id_by_name = {}
        self.link_id_by_name = {}

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    @property
    def robot(self):
        return self._robot

    @robot.setter
    def robot(self, robot):
        self._robot = robot
        self._robot.client = self
        self.load_robot_from_urdf(robot.model.urdf_filename)
        self.create_collision_map()

    def step_simulation(self):
        pybullet.stepSimulation(physicsClientId=self.client_id)

    def load_robot_from_urdf(self, urdf_filename):
        """Create a pybullet robot using the input urdf file.

        Parameters
        ----------
        urdf_filename : :obj:`str`
            Absolute file path to the urdf file. The mesh file can be linked by either
            `"package::"` or relative path.
        """
        robot_model = RobotModel.from_urdf_file(urdf_filename)
        self._robot = Robot(robot_model)
        self._robot.client = self
        self.create_collision_map()
        with redirect_stdout(enabled=not self.verbose):
            self.robot_uid = pybullet.loadURDF(urdf_filename, useFixedBase=True, physicsClientId=self.client_id)
        self.create_name_id_maps()

    def create_name_id_maps(self):
        self.joint_id_by_name = {}
        self.link_id_by_name = {}
        joint_ids = self._get_joint_ids(self.robot_uid)
        for joint_id in joint_ids:
            link_name = self._get_link_name(joint_id, self.robot_uid)
            self.link_id_by_name[link_name] = joint_id
            joint_name = self._get_joint_name(joint_id, self.robot_uid)
            self.joint_id_by_name[joint_name] = joint_id

    def create_collision_map(self, disabled_collisions=None):
        link_names = self.robot.get_link_names_with_collision_geometry()
        disabled_collisions = disabled_collisions or set()
        if self.robot.semantics:
            disabled_collisions.update(self.robot.semantics.disabled_collisions)
        self.collision_map = {}
        for link1, link2 in combinations(link_names, 2):
            if {link1, link2} in disabled_collisions:
                continue
            self.collision_map.setdefault(link1, []).append(link2)

    def remove_configurations_in_collision(self, configurations):
        """Used for a custom inverse kinematics function.
        """
        for i, configuration in enumerate(configurations):
            if not configuration:  # if an ik solution was already removed
                continue
            detected_collision = self.check_collisions(configuration)
            if detected_collision.in_collision:
                configurations[i] = None

    def check_collisions(self, configuration=None):
        self.ensure_robot()
        if configuration:
            joint_ids = tuple(self.joint_id_by_name[name] for name in configuration.joint_names)
            self._set_joint_positions(joint_ids, configuration.values, self.robot_uid)
        detected_collision = self._check_robot_self_collision()
        if detected_collision.in_collision:
            return detected_collision
        detected_collision = self._check_collision_with_objects()
        if detected_collision.in_collision:
            return detected_collision
        detected_collision = self._check_collision_with_attached_objects()
        if detected_collision.in_collision:
            return detected_collision

        return DetectedCollision(False, (None, None))

    def _check_collision_with_attached_objects(self):
        for name, constraint_info in self.attached_collision_objects.items():
            pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=constraint_info.body_id, distance=0, physicsClientId=self.client_id)
            if pts:
                LOG.warning("Collision between 'robot' and '{}'".format(name))
            for collision_object_name, body_ids in self.collision_objects.items():
                for body_id in body_ids:
                    pts = pybullet.getClosestPoints(bodyA=body_id, bodyB=constraint_info.body_id, distance=0, physicsClientId=self.client_id)
                    if pts:
                        LOG.warning("Collision between '{}' and '{}'".format(name, collision_object_name))
                        return DetectedCollision(True, (name, collision_object_name))
        return DetectedCollision(False, (None, None))

    def _check_collision_with_objects(self):
        for name, body_ids in self.collision_objects.items():
            for body_id in body_ids:
                pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=body_id, distance=0, physicsClientId=self.client_id)
                if pts:
                    LOG.warning("Collision between 'robot' and '{}'".format(name))
                    return DetectedCollision(True, ("robot", name))
        return DetectedCollision(False, (None, None))

    def _check_robot_self_collision(self):
        for link1_name, names in self.collision_map.items():
            link1 = self.link_id_by_name[link1_name]
            for link2_name in names:
                link2 = self.link_id_by_name[link2_name]
                pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=self.robot_uid, distance=0,
                                                linkIndexA=link1, linkIndexB=link2, physicsClientId=self.client_id)
                if pts:
                    LOG.warning("Collision between '{}' and '{}'".format(link1_name, link2_name))
                    return DetectedCollision(True, (link1_name, link2_name))
        return DetectedCollision(False, (None, None))

    def check_collision_objects_for_collision(self):
        names = self.collision_objects.keys()
        for name1, name2 in combinations(names, 2):
            for body1_id in self.collision_objects[name1]:
                for body2_id in self.collision_objects[name2]:
                    pts = pybullet.getClosestPoints(bodyA=body1_id, bodyB=body2_id, distance=0, physicsClientId=self.client_id)
                    if len(pts):
                        LOG.warning("Collision between '{}' and '{}'".format(name1, name2))
                        return DetectedCollision(True, (name1, name2))
        return DetectedCollision(False, (None, None))

    def ensure_robot(self):
        """Checks if the robot is loaded."""
        if self.robot_uid is None:
            raise Exception('This method is only callable once a robot is loaded')

    def _body_id_or_default(self, body_id):
        if body_id is None:
            self.ensure_robot()
            return self.robot_uid
        return body_id

    def _get_base_frame(self, body_id=None):
        body_id = self._body_id_or_default(body_id)
        pose = pybullet.getBasePositionAndOrientation(body_id, physicsClientId=self.client_id)
        return frame_from_pose(pose)

    def _get_base_name(self, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return self._get_body_info(body_id).base_name.decode(encoding='UTF-8')

    def _get_link_state(self, link_id, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return const.LinkState(*pybullet.getLinkState(body_id, link_id, physicsClientId=self.client_id))

    def _get_body_info(self, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return const.BodyInfo(*pybullet.getBodyInfo(body_id, physicsClientId=self.client_id))

    def _get_joint_info(self, joint_id, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return const.JointInfo(*pybullet.getJointInfo(body_id, joint_id, physicsClientId=self.client_id))

    def _get_num_joints(self, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return pybullet.getNumJoints(body_id, physicsClientId=self.client_id)

    def _get_joint_ids(self, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return list(range(self._get_num_joints(body_id)))

    def _get_joint_name(self, joint_id, body_id=None):
        body_id = self._body_id_or_default(body_id)
        return self._get_joint_info(joint_id, body_id).jointName

    def _get_link_name(self, link_id, body_id=None):
        body_id = self._body_id_or_default(body_id)
        if link_id == const.BASE_LINK_ID:
            return self._get_base_name(body_id)
        return self._get_joint_info(link_id, body_id).linkName.decode('UTF-8')

    def _get_link_frame(self, link_id, body_id=None):
        body_id = self._body_id_or_default(body_id)
        if link_id == const.BASE_LINK_ID:
            return self._get_base_frame(body_id)
        link_state = self._get_link_state(link_id, body_id)
        pose = (link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation)
        return frame_from_pose(pose)

    # =======================================
    def _set_base_frame(self, frame, body_id=None):
        body_id = self._body_id_or_default(body_id)
        point, quaternion = pose_from_frame(frame)
        pybullet.resetBasePositionAndOrientation(body_id, point, quaternion, physicsClientId=self.client_id)

    def _set_joint_position(self, joint_id, value, body_id=None):
        body_id = self._body_id_or_default(body_id)
        pybullet.resetJointState(body_id, joint_id, value, targetVelocity=0, physicsClientId=self.client_id)
        # self.create_collision_map() ???

    def _set_joint_positions(self, joints, values, body_id=None):
        body_id = self._body_id_or_default(body_id)
        if len(joints) != len(values):
            raise Exception('Joints and values must have the same length.')
        for joint, value in zip(joints, values):
            self._set_joint_position(joint, value, body_id)

    # =======================================
    def convert_mesh_to_body(self, mesh, frame, _name=None):
        """Convert compas mesh and its frame to a pybullet body

        Parameters
        ----------
        mesh : `compas.datastructures.Mesh`
        frame : :class:`compas.geometry.Frame`
        _name : :obj:`str`, optional
            Name of the mesh for tagging in pybullet's GUI

        Returns
        -------
        :obj:`int`
        """
        temp_dir = tempfile.mkdtemp()
        tmp_obj_path = os.path.join(temp_dir, 'temp.obj')
        try:
            mesh.to_obj(tmp_obj_path)
            pyb_body_id = self.body_from_obj(tmp_obj_path)
            self._set_base_frame(frame, pyb_body_id)
            # !!! The following lines are apparently for visual debugging purposes
            # To be deleted or rewritten later.
            # if name:
            #     pybullet_planning.add_body_name(pyb_body, name)
        finally:
            os.remove(tmp_obj_path)
            os.rmdir(temp_dir)
        return pyb_body_id

    def body_from_obj(self, path, scale=1., mass=const.STATIC_MASS, collision=True, color=const.GREY):
        geometry_args = self.get_geometry_args(path, scale=scale)

        collision_args = self.get_collision_args(geometry_args)
        collision_id = self.create_collision_shape(collision_args) if collision else const.NULL_ID

        visual_args = self.get_visual_args(geometry_args, color=color)
        visual_id = self.create_visual_shape(visual_args)

        body_id = self.create_body(collision_id, visual_id, mass=mass)
        return body_id

    def create_body(self, collision_id=const.NULL_ID, visual_id=const.NULL_ID, mass=const.STATIC_MASS):
        return pybullet.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id, physicsClientId=self.client_id)

    def create_collision_shape(self, collision_args):
        return pybullet.createCollisionShape(**collision_args)

    def create_visual_shape(self, visual_args):
        if visual_args.get('rgbaColor') is None:
            return const.NULL_ID
        return pybullet.createVisualShape(**visual_args)

    def get_visual_args(self, geometry_args, frame=Frame.worldXY(), color=const.RED, specular=None):
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

    def get_collision_args(self, geometry_args, frame=Frame.worldXY()):
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
    def get_geometry_args(path, scale=1.):
        return {
            'shapeType': pybullet.GEOM_MESH,
            'fileName': path,
            'meshScale': [scale] * 3,
        }
