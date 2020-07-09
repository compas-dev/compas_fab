from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.vrep.helpers import assert_robot
from compas_fab.backends.vrep.helpers import DEFAULT_OP_MODE
from compas_fab.backends.vrep.helpers import VrepError
from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.backends.vrep.remote_api import vrep

__all__ = [
    'VrepAddAttachedCollisionMesh',
]


class VrepAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a building member to the 3D scene and attach it to the robot.
    """
    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Adds a building member to the 3D scene and attaches it to the robot.

        Args:
            attached_collision_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.
            options (:obj:`dict`): Dictionary containing the following key-value pairs:

                - ``"robot_name'``: (:obj:`str`) Name of robot instance to attach the building member to.


        Returns:
            int: Object handle (identifier) assigned to the building member.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        options = options or {}
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
        handles = self.client.add_collision_mesh([building_member_mesh])

        if len(handles) != 1:
            raise VrepError('Expected one handle, but multiple found=' + str(handles), -1)

        handle = handles[0]

        parent_handle = self.client.get_object_handle('customGripper' + robot_name + '_connection')
        vrep.simxSetObjectParent(self.client.client_id, handle, parent_handle, True, DEFAULT_OP_MODE)

        return handle

    def pick_building_member(self, robot, building_member_mesh, pickup_frame, metric_values=None):
        """Picks up a building member and attaches it to the robot.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance to use for pick up.
            building_member_mesh (:class:`compas.datastructures.Mesh`): Mesh
                of the building member that will be attached to the robot.
            pickup_frame (:class:`Frame`): Pickup frame.
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

        self.client.set_robot_pose(robot, pickup_frame)

        return self.add_attached_collision_mesh(building_member_mesh, options={'robot_name': robot.name})
