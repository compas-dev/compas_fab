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
    >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    >>> cm = CollisionMesh(mesh, 'tip')
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
        
        Returns
        -------
        None

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
        >>> time.sleep(1.) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        
        collision_mesh.root_name = self.robot.root_link_name
        
        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.client.add_collision_mesh(collision_mesh)

    def remove_collision_mesh(self, id):
        """Removes a collision object from the planning scene.

        Parameters
        ----------
        id : str
            The identifier of the collision object.
        
        Returns
        -------
        None

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
        >>> time.sleep(1.) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        self.robot.client.remove_collision_mesh(id)

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
        
        Returns
        -------
        None

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
        >>> scene.append_collision_mesh(cm)
        >>> time.sleep(1.) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_link_name

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.robot.client.append_collision_mesh(collision_mesh)
    
    def add_attached_collision_mesh(self, attached_collision_mesh):
        """Adds an attached collision object to the planning scene.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`

        Returns
        -------
        None

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
        >>> # create collison object
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
        >>> cm = CollisionMesh(mesh, 'tip')
        >>> # get robot link names
        >>> robot = Robot()
        >>> ee_link_name = robot.get_end_effector_link_name()
        >>> link_names = robot.get_link_names()
        >>> # the collision mesh is allowed to collide with the last 2 links
        >>> touch_links = link_names[-2:]
        >>> # create a collision mesh attached to the end-effector
        >>> acm = AttachedCollisionMesh(cm, ee_link_name, touch_links)
        >>> # attach it to the end-effector
        >>> scene.add_attached_collision_mesh(acm)
        >>> time.sleep(2)
        >>> scene.remove_attached_collision_mesh('tip')
        >>> # if you completely want to remove the collison mesh, you also have
        >>> # to remove it from the scene
        >>> scene.remove_collision_mesh('tip')
        >>> time.sleep(2) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        self.client.add_attached_collision_mesh(attached_collision_mesh)
    
    def remove_attached_collision_mesh(self, id):
        """Removes an attached collision object to the planning scene.

        Parameters
        ----------
        id : str
            The identifier of the object.
        
        Returns
        -------
        None

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
        >>> # create collison object
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
        >>> cm = CollisionMesh(mesh, 'tip')
        >>> # get robot link names
        >>> robot = Robot()
        >>> ee_link_name = robot.get_end_effector_link_name()
        >>> link_names = robot.get_link_names()
        >>> # the collision mesh is allowed to collide with the last 2 links
        >>> touch_links = link_names[-2:]
        >>> # create a collision mesh attached to the end-effector
        >>> acm = AttachedCollisionMesh(cm, ee_link_name, touch_links)
        >>> # attach it to the end-effector
        >>> scene.add_attached_collision_mesh(acm)
        >>> time.sleep(2)
        >>> scene.remove_attached_collision_mesh('tip')
        >>> # if you completely want to remove the collison mesh, you also have
        >>> # to remove it from the scene
        >>> scene.remove_collision_mesh('tip')
        >>> time.sleep(2) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        self.client.remove_attached_collision_mesh(id)

    def attach_collision_mesh_to_end_effector(self, collision_mesh, scale=False, group=None):
        """Attaches a collision mesh to the robot's end-effector.

        Parameters
        ----------
        collision_mesh: :class:`compas_fab.robots.CollisionMesh`
            The collision mesh.
        scale : bool, optional
            If `True`, the mesh will be scaled according to the robot's scale 
            factor.
        group : str
            The planning group to which we want to attach the mesh to. Defaults
            to the robot's main planning group.
        
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
        >>> # create collison object
        >>> mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
        >>> cm = CollisionMesh(mesh, 'tip')
        >>> # attach it to the end-effector
        >>> group = robot.main_group_name
        >>> scene.attach_collision_mesh_to_end_effector(cm, group="group")
        >>> time.sleep(2)
        >>> scene.remove_attached_collision_mesh('tip')
        >>> # if you completely want to remove the collison mesh, you also have
        >>> # to remove it from the scene
        >>> scene.remove_collision_mesh('tip')
        >>> time.sleep(2) #sleep a bit before terminating the client
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)
        
        ee_link_name = self.robot.get_end_effector_link_name(group)
        touch_links = [ee_link_name]
        acm = AttachedCollisionMesh(collision_mesh, ee_link_name, touch_links)
        self.add_attached_collision_mesh(acm)



if __name__ == "__main__":

    #import doctest
    #doctest.testmod()
    import time
    print([name for name in dir() if not name.startswith('_')])
    from compas_fab.robots.ur5 import Robot
    robot = Robot()
    print(robot.root_link_name)

    import compas_fab
    from compas.datastructures import Mesh
    from compas_fab.robots.ur5 import Robot
    from compas_fab.robots import PlanningScene
    from compas_fab.robots import CollisionMesh
    from compas_fab.backends import RosClient
    client = RosClient()
    client.run()
    robot = Robot(client)
    scene = PlanningScene(robot)
    mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    cm = CollisionMesh(mesh, 'tip')
    scene.attach_collision_mesh_to_end_effector(cm)
    time.sleep(2)
    scene.remove_attached_collision_mesh('tip')
    scene.remove_collision_mesh('tip')
    time.sleep(2)
    client.close()
    client.terminate()