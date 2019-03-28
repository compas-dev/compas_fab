from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Scale
from compas.geometry import Frame

from compas.datastructures import mesh_transform


__all__ = [
    'CollisionMesh',
    'AttachedCollisionMesh',
    'PlanningScene',
]

class CollisionMesh(object):
    """Represents a collision mesh.

    Attributes
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The collision mesh. Ideally it is as coarse as possible.
    id : str
        The id of the mesh, used to identify it for later operations (remove, 
        append, etc.)
    frame : :class:`compas.geometry.Frame`, optional
        The frame of the mesh. Defaults to the world XY frame.
    root_name : str
        The name of the root link the collision mesh with be placed in. Defaults
        to 'world'.
    
    Examples
    --------
    >>> import compas_fab
    >>> from compas.datastructures import Mesh
    >>> from compas_fab.robots import CollisionMesh
    >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    >>> cm = CollisionMesh(mesh, 'floor')
    """

    def __init__(self, mesh, id, frame=None, root_link_name=None):
        self.id = id
        self.mesh = mesh
        self.frame = frame if frame else Frame.worldXY()
        self.root_name = 'world'
    
    def scale(self, transformation):
        """Scales the collision mesh.

        Parameters
        ----------
        transformation : compas.geometry.Scale
        """
        mesh_transform(self.mesh, transformation)


class AttachedCollisionMesh(object):
    """Represents a collision mesh that is attached to a robot's link.

    Attributes
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh we want to attach.
    link_name : str
        The name of the link the collision mesh will be attached to.
    touch_links : list of str
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link_name it is attached to.
    weight : float
        The weight of the attached object. Defaults to 1.
    
    Examples
    --------
    >>> import compas_fab
    >>> from compas.datastructures import Mesh
    >>> from compas_fab.robots import CollisionMesh
    >>> from compas_fab.robots.ur5 import Robot
    >>> # create the collision mesh
    >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    >>> cm = CollisionMesh(mesh, 'floor')
    >>> # get robot link names
    >>> robot = Robot()
    >>> ee_link_name = robot.get_end_effector_link_name()
    >>> link_names = robot.get_link_names()
    >>> # the collision mesh is allowed to collide with the last 2 links
    >>> touch_links = link_names[-2:]
    >>> # create a collision mesh attached to the end-effector
    >>> acm = AttachedCollisionMesh(cm, ee_link_name, touch_links)
    """
    def __init__(self, collision_mesh, link_name, touch_links=None, weight=1.):
        self.collision_mesh = collision_mesh
        self.collision_mesh.root_name = link_name
        self.link_name = link_name
        self.touch_links = touch_links if touch_links else [link_name]
        self.weight = weight


class PlanningScene(object):
    """Represents the planning scene.

    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`
        A reference to the robot in the planning scene. 

    Examples
    --------
    >>> import compas_fab
    >>> from compas.datastructures import Mesh
    >>> from compas_fab.robots.ur5 import Robot
    >>> from compas_fab.robots import PlanningScene
    >>> from compas_fab.robots import CollisionMesh
    >>> from compas_fab.backends import RosClient
    >>> client = RosClient()
    >>> client.run()
    >>> robot = Robot(client)
    >>> scene = PlanningScene(robot)
    >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    >>> cm = CollisionMesh(mesh, 'floor')
    >>> scene.add_collision_mesh(cm)
    >>> client.close()
    >>> client.terminate()
    """

    def __init__(self, robot):
        self.robot = robot
    
    @property
    def client(self):
        """The backend client."""
        return self.robot.client
    
    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')
    
    def add_collision_mesh(self, collision_mesh, scale=False):
        """Adds a collision mesh to the planning scene.

        If the object with the same name previously existed, it is replaced.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            The collision mesh we want to add.
        scale : bool, optional
            If `True`, the mesh will be scaled according to the robot's scale 
            factor.

        Examples
        --------
        >>> import compas_fab
        >>> from compas.datastructures import Mesh
        >>> from compas_fab.robots.ur5 import Robot
        >>> from compas_fab.robots import PlanningScene
        >>> from compas_fab.robots import CollisionMesh
        >>> from compas_fab.backends import RosClient
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.add_collision_mesh(cm)
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        
        collision_mesh.root_name = self.robot.root_link_name
        
        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.client.add_collision_mesh(collision_mesh)

    def remove_collision_mesh(self, name):
        """Removes a collision object from the planning scene.

        Parameters
        ----------
        name : str
            The identifier of the collision object.

        Examples
        --------
        >>> import time
        >>> import compas_fab
        >>> from compas.datastructures import Mesh
        >>> from compas_fab.robots.ur5 import Robot
        >>> from compas_fab.robots import PlanningScene
        >>> from compas_fab.robots import CollisionMesh
        >>> from compas_fab.backends import RosClient
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.add_collision_mesh(cm)
        >>> time.sleep(5.)
        >>> scene.remove_collision_mesh('floor')
        >>> client.close()
        >>> client.terminate()
        """
        root_link_name = self.robot.root_link_name # needed?
        self.robot.client.remove_collision_mesh(name, root_link_name)

    def append_collision_mesh(self, collision_mesh, scale=False):
        """Appends a collision mesh that already exists in the planning scene.

        If the does not exist, it is added.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            The collision mesh we want to append.
        scale : bool, optional
            If `True`, the mesh will be scaled according to the robot's scale 
            factor.

        Examples
        --------
        >>> import compas_fab
        >>> from compas.datastructures import Mesh
        >>> from compas_fab.robots.ur5 import Robot
        >>> from compas_fab.robots import PlanningScene
        >>> from compas_fab.robots import CollisionMesh
        >>> from compas_fab.backends import RosClient
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> scene = PlanningScene(robot)
        >>> 
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.append_collision_mesh(cm)
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_link_name

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.robot.client.append_collision_mesh(collision_mesh)
    
    def create_collision_mesh_attached_to_end_effector(self, id_name, mesh, group=None, scale=False, touch_links=None):
        """Creates a collision object that is added to the end effector's tcp.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)

        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        last_link_with_geometry = self.get_links_with_geometry(group)[-1]
        if not touch_links:
            touch_links=[last_link_with_geometry.name]
        else:
            touch_links = list(touch_links)
            if last_link_with_geometry.name not in touch_links:
                touch_links.append(last_link_with_geometry.name)

        return self.client.build_attached_collision_mesh(ee_link_name, id_name, mesh, operation=0, touch_links=touch_links)

    def add_attached_collision_mesh(self, id_name, mesh, group=None, touch_links=[], scale=False):
        """Attaches a collision mesh to the robot's end-effector.

        Parameters
        ----------
            id_name (str): The identifier of the object.
            mesh (:class:`Mesh`): A triangulated COMPAS mesh.
            group (str, optional): The planning group on which's end-effector
                the object should be attached. Defaults to the robot's main
                planning group.
            touch_links(str list): The list of link names that the attached mesh
                is allowed to touch by default. The end-effector link name is
                already considered.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)

        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        if ee_link_name not in touch_links:
            touch_links.append(ee_link_name)

        self.client.attached_collision_mesh(id_name, ee_link_name, mesh, 0, touch_links)

    def remove_attached_collision_mesh(self, id_name, group=None):
        """Removes an attached collision object from the robot's end-effector.

        Parameters
        ----------
            id_name (str): The identifier of the object.
            group (str, optional): The planning group on which's end-effector
                the object should be removed. Defaults to the robot's main
                planning group.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)
        self.client.attached_collision_mesh(id_name, ee_link_name, None, 1)


if __name__ == "__main__":

    #import doctest
    #doctest.testmod()
    print([name for name in dir() if not name.startswith('_')])
    from compas_fab.robots.ur5 import Robot
    robot = Robot()
    print(robot.root_link_name)