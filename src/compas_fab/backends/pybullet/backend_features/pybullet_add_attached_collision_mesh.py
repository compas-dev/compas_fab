from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Frame
from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.backends.pybullet.const import BASE_LINK_ID
from compas_fab.backends.pybullet.const import ConstraintInfo
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletAddAttachedCollisionMesh',
]


class PyBulletAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a collision mesh and attach it to the robot."""
    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"max_force"``: (:obj:`float`) The maximum force that
              the constraint can apply. Optional.
            - ``"mass"``: (:obj:`float`) The mass of the object, in kg.
            - ``"robot"``: (:class:`compas_fab.robots.Robot``) Robot instance
              to which the object should be attached.

        Returns
        -------
        ``None``
        """
        options = options or {}
        robot = options['robot']
        mesh = attached_collision_mesh.collision_mesh.mesh
        name = attached_collision_mesh.collision_mesh.id

        robot_tool0_link_id = robot.model.get_link_by_name(attached_collision_mesh.link_name).attr['pybullet']['id']
        robot_tool0_frame = self.client._get_link_frame(robot_tool0_link_id, robot.attributes['pybullet_uid'])
        robot_tool0_link_state = self.client._get_link_state(robot_tool0_link_id, robot.attributes['pybullet_uid'])
        inverted_tool0_com_point, inverted_tool0_com_frame = pybullet.invertTransform(
            robot_tool0_link_state.linkWorldPosition,
            robot_tool0_link_state.linkWorldOrientation
        )

        body_id = self.client.convert_mesh_to_body(mesh, robot_tool0_frame, mass=options['mass'])
        body_link_id = BASE_LINK_ID
        body_point, body_quaternion = pose_from_frame(robot_tool0_frame)

        grasp_point, grasp_quaternion = pybullet.multiplyTransforms(
            inverted_tool0_com_point, inverted_tool0_com_frame,
            body_point, body_quaternion
        )

        constraint_id = pybullet.createConstraint(robot.attributes['pybullet_uid'], robot_tool0_link_id, body_id, body_link_id,
                                                  pybullet.JOINT_FIXED, jointAxis=Frame.worldXY().point,
                                                  parentFramePosition=grasp_point,
                                                  childFramePosition=Frame.worldXY().point,
                                                  parentFrameOrientation=grasp_quaternion,
                                                  childFrameOrientation=Frame.worldXY().quaternion.xyzw,
                                                  physicsClientId=self.client.client_id)
        if options.get('max_force') is not None:
            pybullet.changeConstraint(constraint_id, maxForce=options['max_force'], physicsClientId=self.client.client_id)

        # mimic ROS' behavior: collision object with same name is replaced
        if name in self.client.attached_collision_objects:
            self.client.remove_attached_collision_mesh(name)

        constraint_info = ConstraintInfo(constraint_id, body_id, robot.attributes['pybullet_uid'])
        self.client.attached_collision_objects[name] = [constraint_info]
