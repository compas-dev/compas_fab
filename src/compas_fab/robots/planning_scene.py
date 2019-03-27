from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Scale

from compas.datastructures import mesh_transformed


__all__ = [
    'PlanningScene',
]

class PlanningScene(object):
    """

    Examples
    --------
    >>> scene = PlanningScene(client)
    >>> scene.add_collision_mesh(name, mesh, replace=True)
    >>> scene.add_collision_mesh(name, mesh)
    """

    def __init__(self, robot, client):
        self.robot = robot
        self.client = client
    
    def add_collision_mesh(self, name, mesh, scale=False):
        """Adds a collision mesh to the robot's planning scene.

        Parameters
        ----------
        name : str
            The identifier of the collision mesh.
        mesh : :class:`compas.datastructures.Mesh`
            A triangulated COMPAS mesh.

        Examples
        --------
        """
        root_link_name = self.robot.root_link_name

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        self.client.collision_mesh(name, root_link_name, mesh, 0)

    def remove_collision_mesh(self, name):
        """Removes a collision mesh from the robot's planning scene.

        Parameters
        ----------
        name : str
            The identifier of the collision mesh.

        Examples
        --------
        """
        root_link_name = self.robot.root_link_name
        self.robot.client.collision_mesh(name, root_link_name, None, 1)

    def append_collision_mesh(self, name, mesh, scale=False):
        """Appends a collision mesh to the robot's planning scene.

        Parameters
        ----------
        name : str
            The identifier of the collision mesh.
        mesh : :class:`compas.datastructures.Mesh`
            A triangulated COMPAS mesh.

        Examples
        --------
        """
        root_link_name = self.robot.root_link_name
        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        self.client.collision_mesh(name, root_link_name, mesh, 2)