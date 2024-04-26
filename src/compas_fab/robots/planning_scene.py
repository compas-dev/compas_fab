from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.data import Data
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Scale

__all__ = [
    "AttachedCollisionMesh",
    "CollisionMesh",
    "PlanningScene",
]


class CollisionMesh(Data):
    """Represents a collision mesh.

    Parameters
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The collision mesh. Ideally it is as coarse as possible.
    id : :obj:`str`
        The id of the mesh, used to identify it for later operations
        (:meth:`~PlanningScene.add_collision_mesh`,
        :meth:`~PlanningScene.remove_collision_mesh`,
        :meth:`~PlanningScene.append_collision_mesh` etc.)
    frame : :class:`compas.geometry.Frame`, optional
        The frame of the mesh. Defaults to :meth:`compas.geometry.Frame.worldXY`.
    root_name : :obj:`str`
        The name of the root link the collision mesh will be placed in. Defaults
        to ``'world'``.

    Attributes
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The collision mesh. Ideally it is as coarse as possible.
    id : :obj:`str`
        The id of the mesh, used to identify it for later operations
        (:meth:`~PlanningScene.add_collision_mesh`,
        :meth:`~PlanningScene.remove_collision_mesh`,
        :meth:`~PlanningScene.append_collision_mesh` etc.)
    frame : :class:`compas.geometry.Frame`, optional
        The frame of the mesh. Defaults to :meth:`compas.geometry.Frame.worldXY`.
    root_name : :obj:`str`
        The name of the root link the collision mesh will be placed in. Defaults
        to ``'world'``.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    >>> cm = CollisionMesh(mesh, 'floor')
    >>> cm.frame
    Frame(point=Point(x=0.0, y=0.0, z=0.0), xaxis=Vector(x=1.0, y=0.0, z=0.0), yaxis=Vector(x=0.0, y=1.0, z=0.0))
    """

    def __init__(self, mesh, id, frame=None, root_name=None):
        super(CollisionMesh, self).__init__()
        self.id = id
        self.mesh = mesh
        self.frame = frame or Frame.worldXY()
        self.root_name = root_name or "world"

    def scale(self, scale_factor):
        """Scales the collision mesh uniformly.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor.
        """
        S = Scale.from_factors([scale_factor] * 3)
        self.mesh.transform(S)

    def scaled(self, scale_factor):
        """Copies the collision mesh, and scales the copy uniformly.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor.
        """
        self.mesh = self.mesh.copy()
        self.scale(scale_factor)

    @classmethod
    def __from_data__(cls, data):
        """Construct a collision mesh from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`CollisionMesh`
             An instance of :class:`CollisionMesh`.
        """
        id = data["id"]
        mesh = Mesh.__from_data__(data["mesh"])
        frame = Frame.__from_data__(data["frame"])
        root_name = data["root_name"]

        collision_mesh = cls(mesh, id, frame, root_name)
        return collision_mesh

    @property
    def __data__(self):
        """:obj:`dict` : The data representing the collision mesh."""
        data_obj = {}
        data_obj["id"] = self.id
        data_obj["mesh"] = self.mesh.__data__
        data_obj["frame"] = self.frame.__data__
        data_obj["root_name"] = self.root_name

        return data_obj


class AttachedCollisionMesh(Data):
    """Represents a collision mesh that is attached to a :class:`Robot`'s :class:`compas_robots.model.Link`.

    Parameters
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh to be attached to the robot model.
    link_name : :obj:`str`
        The name of the :class:`compas_robots.model.Link` the collision mesh will be
        attached to.
    touch_links : :obj:`list` of :obj:`str`, optional
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link it is attached to.
    weight : :obj:`float`, optional
        The weight of the attached object in kg. Defaults to ``1.0``.

    Attributes
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh we want to attach.
    link_name : :obj:`str`
        The name of the :class:`compas_robots.model.Link` the collision mesh will be
        attached to.
    touch_links : :obj:`list` of :obj:`str`
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link it is attached to.
    weight : :obj:`float`
        The weight of the attached object in kg.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> cm = CollisionMesh(mesh, 'tip')
    >>> end_effector_link_name = 'tool0'
    >>> touch_links = ['wrist_3_link', 'tool0']
    >>> acm = AttachedCollisionMesh(cm, end_effector_link_name, touch_links)
    """

    def __init__(self, collision_mesh, link_name, touch_links=None, weight=1.0):
        super(AttachedCollisionMesh, self).__init__()
        self.collision_mesh = collision_mesh
        if self.collision_mesh:
            self.collision_mesh.root_name = link_name
        self.link_name = link_name
        self.touch_links = touch_links if touch_links else [link_name]
        self.weight = weight

    @classmethod
    def __from_data__(cls, data):
        """Construct an attached collision mesh from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`AttachedCollisionMesh`
             An instance of :class:`AttachedCollisionMesh`.
        """
        collision_mesh = CollisionMesh.__from_data__(data["collision_mesh"])
        link_name = data["link_name"]
        touch_links = data["touch_links"]
        weight = data["weight"]

        acm = cls(collision_mesh, link_name, touch_links, weight)
        return acm

    @property
    def __data__(self):
        """:obj:`dict` : The data representing the attached collision mesh."""
        data_obj = {}
        data_obj["collision_mesh"] = self.collision_mesh.__data__
        data_obj["link_name"] = self.link_name
        data_obj["touch_links"] = self.touch_links
        data_obj["weight"] = self.weight

        return data_obj


class PlanningScene(object):
    """Represents the planning scene.

    Parameters
    ----------
    robot : :class:`Robot`
        A reference to the robot in the planning scene.

    Attributes
    ----------
    robot : :class:`Robot`
        A reference to the robot in the planning scene.
    client
        A reference to the robot's backend client.
    """

    def __init__(self, robot):
        self.robot = robot

    @property
    def client(self):
        """:class:`compas_fab.backend.ClientInterface`: The backend client."""
        return self.robot.client

    def ensure_client(self):
        """Ensure that the planning scene's robot has a defined client.

        Raises
        ------
        :exc:`Exception`
            If no client is set for planning scene's robot.
        """
        if not self.client:
            raise Exception("This method is only callable once a client is assigned")

    def reset(self):
        """Resets the planning scene, removing all added collision meshes."""
        self.ensure_client()
        self.client.reset_planning_scene()

    def add_collision_mesh(self, collision_mesh, scale=False):
        """Add a collision mesh to the planning scene.

        If there is already a :class:`CollisionMesh` in the
        :class:`PlanningScene` with the same `id` it will be replaced.

        Parameters
        ----------
        collision_mesh : :class:`CollisionMesh`
            The collision mesh we want to add.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be copied and scaled according to
            the robot's scale factor.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        ...     cm = CollisionMesh(mesh, 'floor')
        ...     scene.add_collision_mesh(cm)                   # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            scale_factor = 1.0 / self.robot.scale_factor
            collision_mesh.scaled(scale_factor)

        self.client.add_collision_mesh(collision_mesh)

    def remove_collision_mesh(self, id):
        """Remove a collision object from the planning scene.

        Parameters
        ----------
        id : :obj:`str`
            The `id` of the :class:`CollisionMesh` instance to remove.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     scene.remove_collision_mesh('floor')           # doctest: +SKIP
        """
        self.ensure_client()
        self.robot.client.remove_collision_mesh(id)

    def append_collision_mesh(self, collision_mesh, scale=False):
        """Append a collision mesh to the planning scene.

        Appends a :class:`CollisionMesh` to the :class:`PlanningScene` using
        `id` as an identifier of a group or cluster of collision meshes. If the group
        does not exist, it will be created implicitly; if it does exist, the meshes will be
        appended to it instead.

        Grouping meshes under a common identifier allows to remove them all
        in one operation, using the :meth:`~PlanningScene.remove_collision_mesh` with
        the group identifier.

        Parameters
        ----------
        collision_mesh : :class:`CollisionMesh`
            The collision mesh we want to append to the :class:`PlanningScene`.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be copied and scaled according to
            the robot's scale factor.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        ...     cm = CollisionMesh(mesh, 'floor')
        ...     scene.append_collision_mesh(cm)                # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            scale_factor = 1.0 / self.robot.scale_factor
            collision_mesh.scaled(scale_factor)

        self.robot.client.append_collision_mesh(collision_mesh)

    def add_attached_collision_mesh(self, attached_collision_mesh, scale=False):
        """Add an attached collision object to the planning scene.

        Parameters
        ----------
        attached_collision_mesh : :class:`AttachedCollisionMesh`
            The :class:`AttachedCollisionMesh` (a :class:`CollisionMesh`
            attached to a :class:`Robot`'s :class:`compas_robots.model.Link`) that
            we want to add to the :class:`PlanningScene`.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be copied and scaled according to
            the robot's scale factor.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        ...     cm = CollisionMesh(mesh, 'tip')
        ...     end_effector_link_name = 'tool0'
        ...     touch_links = ['wrist_3_link', 'tool0']
        ...     acm = AttachedCollisionMesh(cm, end_effector_link_name, touch_links)
        ...     scene.add_attached_collision_mesh(acm)         # doctest: +SKIP
        """
        self.ensure_client()

        if scale:
            scale_factor = 1.0 / self.robot.scale_factor
            attached_collision_mesh.collision_mesh.scaled(scale_factor)

        self.client.add_attached_collision_mesh(attached_collision_mesh)

    def remove_attached_collision_mesh(self, id):
        """Remove an attached collision object from the planning scene.

        Parameters
        ----------
        id : :obj:`str`
            The `id` of the :class:`CollisionMesh` in the
            :class:`AttachedCollisionMesh` to remove from the
            :class:`PlanningScene`.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     scene.remove_attached_collision_mesh('tip')   # doctest: +SKIP
        """
        self.ensure_client()
        self.client.remove_attached_collision_mesh(id)

    def attach_collision_mesh_to_robot_end_effector(self, collision_mesh, scale=False, group=None):
        """Attaches a collision mesh to the robot's end-effector.

        Parameters
        ----------
        collision_mesh: :class:`CollisionMesh`
            The collision mesh to attach to robot's end effector.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be copied and scaled according to
            the robot's scale factor.
        group : :obj:`str`
            The planning group with the end effector we want to attach the mesh
            to. Defaults to the robot's main planning group.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     scene = PlanningScene(robot)
        ...     mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        ...     cm = CollisionMesh(mesh, 'tip')
        ...     group = robot.main_group_name
        ...     scene.attach_collision_mesh_to_robot_end_effector(cm, group=group)      # attach to ee
        ...     scene.remove_attached_collision_mesh('tip')                             # now detach
        """
        self.ensure_client()

        if scale:
            scale_factor = 1.0 / self.robot.scale_factor
            collision_mesh.scaled(scale_factor)

        end_effector_link_name = self.robot.get_end_effector_link_name(group)
        touch_links = [end_effector_link_name]
        acm = AttachedCollisionMesh(collision_mesh, end_effector_link_name, touch_links)
        self.add_attached_collision_mesh(acm)

    def add_attached_tool(self, tool=None, group=None):
        """Add the robot's attached tool to the planning scene if tool is set."""
        self.ensure_client()
        if tool:
            self.robot.attach_tool(tool, group)

        # robot has 0 or more tools, each tool has a list of attached meshes
        for tool_mesh_list in self.robot.get_attached_tool_collision_meshes():
            for attached_collision_mesh in tool_mesh_list:
                self.add_attached_collision_mesh(attached_collision_mesh)

    def remove_attached_tool(self):
        """Remove the robot's attached tool from the planning scene."""
        self.ensure_client()

        # robot has 0 or more tools, each tool has a list of attached meshes
        for tool_mesh_list in self.robot.get_attached_tool_collision_meshes():
            for attached_collision_mesh in tool_mesh_list:
                self.remove_attached_collision_mesh(attached_collision_mesh.collision_mesh.id)
                self.remove_collision_mesh(attached_collision_mesh.collision_mesh.id)
        self.robot.detach_tool()
