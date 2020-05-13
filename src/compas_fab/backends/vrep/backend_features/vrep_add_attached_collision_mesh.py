from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.vrep.helpers import DEFAULT_OP_MODE
from compas_fab.backends.vrep.helpers import VrepError
from compas_fab.backends.backend_feature_interfaces import AddAttachedCollisionMesh
from compas_fab.backends.vrep.remote_api import vrep

__all__ = [
    'VrepAddAttachedCollisionMesh',
]


class VrepAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options={}):
        """Adds a building member to the 3D scene and attaches it to the robot.

        Args:
            attached_collision_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.
            options (:obj:`dict`): Dictionary containing the following key-value pairs:

                - robot_name (:obj:`str`): Name of robot instance to attach the building member to.


        Returns:
            int: Object handle (identifier) assigned to the building member.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        robot_name = options['robot_name']
        return self.add_building_member(robot_name, attached_collision_mesh)

    def add_building_member(self, robot_name, building_member_mesh):
        """Adds a building member to the 3D scene and attaches it to the robot.

        Args:
            robot_name (:obj:`str`): Name of robot instance to attach the building member to.
            building_member_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.

        Returns:
            int: Object handle (identifier) assigned to the building member.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        handles = self.client.add_meshes([building_member_mesh])

        if len(handles) != 1:
            raise VrepError('Expected one handle, but multiple found=' + str(handles), -1)

        handle = handles[0]

        parent_handle = self.client.get_object_handle('customGripper' + robot_name + '_connection')
        vrep.simxSetObjectParent(self.client.client_id, handle, parent_handle, True, DEFAULT_OP_MODE)

        return handle
