from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import tempfile
from itertools import combinations

import compas
from compas._os import system
import pybullet
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends.interfaces.client import ClientInterface

from .const import BASE_LINK_ID
from .const import BodyInfo
from .const import GREY
from .const import JointInfo
from .const import LinkState
from .const import NULL_ID
from .const import RED
from .const import STATIC_MASS
from .const import ZERO_FRAME
from .planner import PyBulletPlanner
from .utils import LOG
from .utils import redirect_stdout
from .conversions import frame_from_pose
from .conversions import pose_from_frame


__all__ = [
    'PyBulletClient',
]

from ...robots import Robot


class PyBulletBase(object):
    def __init__(self, use_gui):
        self.client_id = None
        self.use_gui = use_gui and not compas_fab.backends._called_from_test

    def connect(self, shadows=True, color=None, width=None, height=None):
        # Shared Memory: execute the physics simulation and rendering in a separate process
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7
        self.detect_display()
        method = pybullet.GUI if self.use_gui else pybullet.DIRECT
        options = self.compose_options(color, width, height)
        with redirect_stdout():
            self.client_id = pybullet.connect(method, options=options)
        if 0 > self.client_id:
            raise Exception('Error in establishing connection with PyBullet.')
        if self.use_gui:
            self.configure_debug_visualizer(shadows)

    def detect_display(self):
        if self.use_gui and system != 'darwin' and not compas.is_windows() and ('DISPLAY' not in os.environ):
            self.use_gui = False
            print('No display detected! Continuing without GUI.')

    @staticmethod
    def compose_options(self, color, width, height):
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
        return self.check_connection() == 1

    def check_connection(self):
        return pybullet.getConnectionInfo(physicsClientId=self.client_id)['isConnected']


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
        # TODO: fuse end effector and robot link tree into one
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
        joint_ids = self.get_joint_ids(self.robot_uid)
        for joint_id in joint_ids:
            link_name = self.get_link_name(joint_id, self.robot_uid)
            self.link_id_by_name[link_name] = joint_id
            joint_name = self.get_joint_name(joint_id, self.robot_uid)
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

    def configuration_in_collision(self, configuration):
        joint_ids = tuple(self.joint_id_by_name[name] for name in configuration.joint_names)
        self.set_joint_positions(joint_ids, configuration.values, self.robot_uid)
        collision, _names = self.collision_check()
        return collision

    def remove_configurations_in_collision(self, configurations):
        """Used for a custom inverse kinematics function.
        """
        for i, configuration in enumerate(configurations):
            if not configuration:  # if an ik solution was already removed
                continue
            in_collision = self.configuration_in_collision(configuration)
            if in_collision:
                configurations[i] = None

    def collision_check(self):
        # collision objects
        for name, body_ids in self.collision_objects.items():
            for body_id in body_ids:
                pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=body_id, distance=0, physicsClientId=self.client_id)
                if pts:
                    LOG.warning("Collision between 'robot' and '{}'".format(name))
                    return True, ("robot", name)

        for link1_name, names in self.collision_map.items():
            link1 = self.link_id_by_name[link1_name]
            for link2_name in names:
                link2 = self.link_id_by_name[link2_name]
                pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=self.robot_uid, distance=0,
                                                linkIndexA=link1, linkIndexB=link2, physicsClientId=self.client_id)
                if pts:
                    LOG.warning("Collision between '{}' and '{}'".format(link1_name, link2_name))
                    return True, (link1_name, link2_name)

        # attached collision objects
        # !!! is this right?
        for name, constraint_info in self.attached_collision_objects.items():
            pts = pybullet.getClosestPoints(bodyA=self.robot_uid, bodyB=constraint_info.body_id, distance=0, physicsClientId=self.client_id)
            if pts:
                LOG.warning("Collision between 'robot' and '{}'".format(name))
            for collision_object_name, body_ids in self.collision_objects.items():
                for body_id in body_ids:
                    pts = pybullet.getClosestPoints(bodyA=body_id, bodyB=constraint_info.body_id, distance=0, physicsClientId=self.client_id)
                    if pts:
                        LOG.warning("Collision between '{}' and '{}'".format(name, collision_object_name))

        return False, ()

    def ensure_robot(self):
        """Checks if the robot is loaded."""
        if self.robot_uid is None:
            raise Exception('This method is only callable once a robot is loaded')

    def body_id_or_default(self, body_id):
        if body_id is None:
            self.ensure_robot()
            return self.robot_uid
        return body_id

    def get_base_frame(self, body_id=None):
        body_id = self.body_id_or_default(body_id)
        pose = pybullet.getBasePositionAndOrientation(body_id, physicsClientId=self.client_id)
        return frame_from_pose(pose)

    def get_base_name(self, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return self.get_body_info(body_id).base_name.decode(encoding='UTF-8')

    def get_link_state(self, link_id, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return LinkState(*pybullet.getLinkState(body_id, link_id, physicsClientId=self.client_id))

    def get_body_info(self, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return BodyInfo(*pybullet.getBodyInfo(body_id, physicsClientId=self.client_id))

    def get_joint_info(self, joint_id, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return JointInfo(*pybullet.getJointInfo(body_id, joint_id, physicsClientId=self.client_id))

    def get_num_joints(self, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return pybullet.getNumJoints(body_id, physicsClientId=self.client_id)

    def get_joint_ids(self, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return list(range(self.get_num_joints(body_id)))

    def get_joint_name(self, joint_id, body_id=None):
        body_id = self.body_id_or_default(body_id)
        return self.get_joint_info(joint_id, body_id).jointName

    def get_link_name(self, link_id, body_id=None):
        body_id = self.body_id_or_default(body_id)
        if link_id == BASE_LINK_ID:
            return self.get_base_name(body_id)
        return self.get_joint_info(link_id, body_id).linkName.decode('UTF-8')

    def get_link_frame(self, link_id, body_id=None):
        body_id = self.body_id_or_default(body_id)
        if link_id == BASE_LINK_ID:
            return self.get_base_frame(body_id)
        link_state = self.get_link_state(link_id, body_id)
        pose = (link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation)
        return frame_from_pose(pose)

    # =======================================
    def set_base_frame(self, frame, body_id=None):
        # !!! This could lead to a discrepancy between the self._robot and what's going on in pybullet.
        # !!! RCF would not match
        body_id = self.body_id_or_default(body_id)
        point, quaternion = pose_from_frame(frame)
        pybullet.resetBasePositionAndOrientation(body_id, point, quaternion, physicsClientId=self.client_id)

    def set_joint_position(self, joint_id, value, body_id=None):
        body_id = self.body_id_or_default(body_id)
        pybullet.resetJointState(body_id, joint_id, value, targetVelocity=0, physicsClientId=self.client_id)
        # self.create_collision_map() ???

    def set_joint_positions(self, joints, values, body_id=None):
        body_id = self.body_id_or_default(body_id)
        if len(joints) != len(values):
            raise Exception('Joints and values must have the same length.')
        for joint, value in zip(joints, values):
            self.set_joint_position(joint, value, body_id)

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
        mesh.to_obj(tmp_obj_path)
        pyb_body_id = self.body_from_obj(tmp_obj_path)
        self.set_base_frame(frame, pyb_body_id)
        # !!! The following lines are apparently for visual debugging purposes
        # To be deleted or rewritten later.
        # if name:
        #     pybullet_planning.add_body_name(pyb_body, name)
        os.remove(tmp_obj_path)
        os.rmdir(temp_dir)
        return pyb_body_id

    def body_from_obj(self, path, scale=1., mass=STATIC_MASS, collision=True, color=GREY):
        geometry_args = self.get_geometry_args(path, scale=scale)

        collision_args = self.get_collision_args(geometry_args)
        collision_id = self.create_collision_shape(collision_args) if collision else NULL_ID

        visual_args = self.get_visual_args(geometry_args, color=color)
        visual_id = self.create_visual_shape(visual_args)

        body_id = self.create_body(collision_id, visual_id, mass=mass)
        return body_id

    def create_body(self, collision_id=NULL_ID, visual_id=NULL_ID, mass=STATIC_MASS):
        return pybullet.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id, physicsClientId=self.client_id)

    def create_collision_shape(self, collision_args):
        return pybullet.createCollisionShape(**collision_args)

    def create_visual_shape(self, visual_args):
        if visual_args.get('rgbaColor') is None:
            return NULL_ID
        return pybullet.createVisualShape(**visual_args)

    def get_visual_args(self, geometry_args, frame=ZERO_FRAME, color=RED, specular=None):
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

    def get_collision_args(self, geometry_args, frame=ZERO_FRAME):
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
    def get_geometry_args(self, path, scale=1.):
        return {
            'shapeType': pybullet.GEOM_MESH,
            'fileName': path,
            'meshScale': [scale] * 3,
        }
