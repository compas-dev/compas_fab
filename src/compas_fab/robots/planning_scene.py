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
            raise Exception(
                'This method is only callable once a client is assigned')

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
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.add_collision_mesh(cm)                   # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            S = Scale([1. / self.robot.scale_factor] * 3)
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
        >>> scene = PlanningScene(robot)
        >>> scene.remove_collision_mesh('floor')           # doctest: +SKIP
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
        >>> scene = PlanningScene(robot)
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        >>> cm = CollisionMesh(mesh, 'floor')
        >>> scene.append_collision_mesh(cm)                # doctest: +SKIP
        """
        self.ensure_client()

        collision_mesh.root_name = self.robot.root_name

        if scale:
            S = Scale([1. / self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        self.robot.client.append_collision_mesh(collision_mesh)

    def add_attached_collision_mesh(self, attached_collision_mesh, scale=False):
        """Adds an attached collision object to the planning scene.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
        scale : bool, optional
            If `True`, the mesh will be scaled according to the robot's scale
            factor.

        Returns
        -------
        None

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
            S = Scale([1. / self.robot.scale_factor] * 3)
            attached_collision_mesh.collision_mesh.scale(S)

        self.client.add_attached_collision_mesh(attached_collision_mesh)

    def remove_attached_collision_mesh(self, id):
        """Removes an attached collision object from the planning scene.

        Parameters
        ----------
        id : str
            The identifier of the object.

        Returns
        -------
        None

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
        collision_mesh: :class:`compas_fab.robots.CollisionMesh`
            The collision mesh.
        scale : bool, optional
            If `True`, the mesh will be scaled according to the robot's scale
            factor.
        group : str
            The planning group to which we want to attach the mesh to. Defaults
            to the robot's main planning group.

        Returns
        -------
        None

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
            S = Scale([1. / self.robot.scale_factor] * 3)
            collision_mesh.scale(S)

        ee_link_name = self.robot.get_end_effector_link_name(group)
        touch_links = [ee_link_name]
        acm = AttachedCollisionMesh(collision_mesh, ee_link_name, touch_links)
        self.add_attached_collision_mesh(acm)

    def add_attached_tool(self):
        """Adds the robot's attached tool to the planning scene if set.
        """
        self.ensure_client()
        if self.robot.attached_tool:
            self.add_attached_collision_mesh(self.robot.attached_tool.attached_collision_mesh)

    def remove_attached_tool(self):
        """Removes the robot's attached tool from the planning scene.
        """
        self.ensure_client()
        if self.robot.attached_tool:
            self.remove_attached_collision_mesh(self.robot.attached_tool.name)

    # TODO:
    # def clear_planning_scene(self):
    # def reset_planning_scene(self, robot_full_configuration):
    # make function to fully reset the planning scene

    def apply_planning_scene(self, robot_full_configuration):
        """http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/ApplyPlanningScene.html
        """
        self.ensure_client()

        # needed?
        from compas_fab.backends.ros.messages import PlanningScene
        from compas_fab.backends.ros.messages import PlanningSceneWorld
        from compas_fab.backends.ros.messages import RobotState
        from compas_fab.backends.ros.messages import JointState

        # scene robot states
        js = JointState(
            header=None,
            name=self.get_configurable_joint_names(),
            position=robot_full_configuration.values)
        print("JointState =", js)
        rs = RobotState(
            joint_state=js,
            multi_dof_joint_state=None,
            attached_collision_objects=None,
            is_diff=True)
        print("RobotState =", rs)

        planning_scene = PlanningScene(
            name='robots_state_at_start',
            robot_state=rs,
            robot_model_name=self.name,
            fixed_frame_transforms=None,
            allowed_collision_matrix=None,
            link_padding=None,
            link_scale=self._scale_factor,
            object_colors=None,
            world=PlanningSceneWorld(),
            is_diff=True)
        #print("planning_scene =", planning_scene)

        response = self.client.apply_planning_scene(planning_scene)
        print("response =", response)
