from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.datastructures import mesh_transform
from compas.geometry import Frame
from compas.geometry import Scale

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
    id : :obj:`str`
        The id of the mesh, used to identify it for later operations
        (:meth:`~PlanningScene.add_collision_mesh`,
        :meth:`~PlanningScene.remove_collision_mesh`,
        :meth:`~PlanningScene.append_collision_mesh` etc.)
    frame : :class:`compas.geometry.Frame`, optional
        The frame of the mesh. Defaults to the world XY frame.
    root_name : :obj:`str`
        The name of the root link the collision mesh will be placed in. Defaults
        to `'world'`.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    >>> cm = CollisionMesh(mesh, 'floor')
    >>> cm.frame
    Frame(Point(0.000, 0.000, 0.000), Vector(1.000, 0.000, 0.000), Vector(0.000, 1.000, 0.000))
    """

    def __init__(self, mesh, id, frame=None, root_name=None):
        self.id = id
        self.mesh = mesh
        self.frame = frame or Frame.worldXY()
        self.root_name = root_name or 'world'

    def scale(self, transformation):
        """Scales the collision mesh.

        Parameters
        ----------
        transformation : :class:`compas.geometry.Scale`
            Scaling transformation to apply to collision mesh.
        """
        mesh_transform(self.mesh, transformation)


class AttachedCollisionMesh(object):
    """Represents a collision mesh that is attached to a :class:`Robot`'s :class:`~compas.robots.Link`.

    Attributes
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh we want to attach.
    link_name : :obj:`str`
        The name of the :class:`~compas.robots.Link` the collision mesh will be
        attached to.
    touch_links : :obj:`list` of :obj:`str`
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link it is attached to.
    weight : :obj:`float`
        The weight of the attached object. Defaults to ``1.0``.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> cm = CollisionMesh(mesh, 'tip')
    >>> ee_link_name = 'ee_link'
    >>> touch_links = ['wrist_3_link', 'ee_link']
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
    robot : :class:`Robot`
        A reference to the robot in the planning scene.
    """

    def __init__(self, robot):
        self.robot = robot

    @property
    def client(self):
        """:class:`compas_fab.backend.RosClient` or :class:`compas_fab.backend.VrepClient` : The backend client."""
        return self.robot.client

    def ensure_client(self):
        """Ensure that the planning scene's robot has a defined client.

        Raises
        ------
        :exc:`Exception`
            If no client is set for planning scene's robot.
        """
        if not self.client:
            raise Exception(
                'This method is only callable once a client is assigned')

    def add_collision_mesh(self, collision_mesh, scale=False):
        """Add a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`CollisionMesh`
            The collision mesh we want to add.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be scaled according to the robot's scale
            factor.

        Returns
        -------
        ``None``

        Note
        ----
        A :class:`CollisionMesh` with the same `id` as an existing
        :class:`CollisionMesh` in the :class:`PlanningScene` will
        replace the existing collision object.

        Examples
        --------
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.add_collision_mesh(cm)                   # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

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
        >>> scene = PlanningScene(robot)
        >>> scene.remove_collision_mesh('floor')           # doctest: +SKIP
        """
        self.ensure_client()
        self.robot.client.remove_collision_mesh(id)

    def append_collision_mesh(self, collision_mesh, scale=False):
        """Append a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`CollisionMesh`
            The collision mesh we want to append to the :class:`PlanningScene`.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be scaled according to the robot's scale
            factor.

        Returns
        -------
        ``None``

        Note
        ----
        If there is already a :class:`CollisionMesh` instance with the same `id`
        in the :class:`PlanningScene`, it will not be replaced.

        Examples
        --------
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.append_collision_mesh(cm)                # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.robot.client.append_collision_mesh(collision_mesh)

    def add_attached_collision_mesh(self, attached_collision_mesh, scale=False):
        """Add an attached collision object to the planning scene.

        Parameters
        ----------
        attached_collision_mesh : :class:`AttachedCollisionMesh`
            The :class:`AttachedCollisionMesh` (a :class:`CollisionMesh`
            attached to a :class:`Robot`'s :class:`~compas.robots.Link`) that
            we want to add to the :class:`PlanningScene`.
        scale : :obj:`bool`, optional
            If ``True``, the mesh will be scaled using the robot's scale factor.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> cm = CollisionMesh(mesh, 'tip')
        >>> ee_link_name = 'ee_link'
        >>> touch_links = ['wrist_3_link', 'ee_link']
        >>> acm = AttachedCollisionMesh(cm, ee_link_name, touch_links)
        >>> scene.add_attached_collision_mesh(acm)         # doctest: +SKIP
        """
        self.ensure_client()

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            attached_collision_mesh.collision_mesh.scale(S)

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
        >>> scene = PlanningScene(robot)
        >>> scene.remove_attached_collision_mesh('tip')   # doctest: +SKIP
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
            If ``True``, the mesh will be scaled using the robot's scale
            factor.
        group : :obj:`str`
            The planning group with the end effector we want to attach the mesh
            to. Defaults to the robot's main planning group.

        Returns
        -------
        ``None``

        Examples
        --------
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> cm = CollisionMesh(mesh, 'tip')
        >>> group = robot.main_group_name
        >>> scene.attach_collision_mesh_to_robot_end_effector(cm, group=group)

        >>> # wait for it to be attached
        >>> time.sleep(1)

        >>> # wait for it to be removed
        >>> scene.remove_attached_collision_mesh('tip')
        >>> time.sleep(1)

        >>> # check if it's really gone
        >>> planning_scene = robot.client.get_planning_scene()
        >>> objects = [c.object['id'] for c in planning_scene.robot_state.attached_collision_objects]
        >>> 'tip' in objects
        False
        """
        self.ensure_client()

        if scale:
            S = Scale([1./self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        ee_link_name = self.robot.get_end_effector_link_name(group)
        touch_links = [ee_link_name]
        acm = AttachedCollisionMesh(collision_mesh, ee_link_name, touch_links)
        self.add_attached_collision_mesh(acm)

    def add_attached_tool(self):
        """Add the robot's attached tool to the planning scene if tool is set."""
        self.ensure_client()
        if self.robot.attached_tool:
            self.add_attached_collision_mesh(self.robot.attached_tool.attached_collision_mesh)

    def remove_attached_tool(self):
        """Remove the robot's attached tool from the planning scene."""
        self.ensure_client()
        if self.robot.attached_tool:
            self.remove_attached_collision_mesh(self.robot.attached_tool.name)
