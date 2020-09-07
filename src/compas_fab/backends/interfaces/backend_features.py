from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta
from abc import abstractmethod


class ForwardKinematics(object):
    """Interface for a Planner's forward kinematics feature.  Any implementation of
    ``ForwardKinematics`` must define the method ``forward_kinematics``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``ForwardKinematics`` to be treated as its ``forward_kinematics`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, configuration, group=None, options=None):
        return self.forward_kinematics(robot, configuration, group, options)

    @abstractmethod
    def forward_kinematics(self, robot, configuration, group=None, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which forward kinematics is being calculated.
        configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : str, optional
            The name of the group to be used in the calculation.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        pass


class InverseKinematics(object):
    """Interface for a Planner's inverse kinematics feature.  Any implementation of
    ``InverseKinematics`` must define the method ``inverse_kinematics``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``InverseKinematics`` to be treated as its ``inverse_kinematics`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        return self.inverse_kinematics(robot, frame_WCF, start_configuration, group, options)

    @abstractmethod
    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
        group: str, optional
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.
        """
        pass


class PlanMotion(object):
    """Interface for a Planner's plan motion feature.  Any implementation of
    ``PlanMotion`` must define the method ``plan_motion``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``PlanMotion`` to be treated as its ``plan_motion`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, goal_constraints, start_configuration=None, group=None, options=None):
        return self.plan_motion(robot, goal_constraints, start_configuration, group, options)

    @abstractmethod
    def plan_motion(self, robot, goal_constraints, start_configuration=None, group=None, options=None):
        """Calculates a motion path.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion path is being calculated.
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The name of the group to plan for.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        pass


class PlanCartesianMotion(object):
    """Interface for a Planner's plan cartesian motion feature.  Any implementation of
    ``PlanCartesianMotion`` must define the method ``plan_cartesian_motion``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``PlanCartesianMotion`` to be treated as its ``plan_cartesian_motion`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, robot, frames_WCF, start_configuration=None, group=None, options=None):
        return self.plan_cartesian_motion(robot, frames_WCF, start_configuration, group, options)

    @abstractmethod
    def plan_cartesian_motion(self, robot, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion path is being calculated.
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        pass


class GetPlanningScene(object):
    """Interface for a Planner's get planning scene feature.  Any implementation of
    ``GetPlanningScene`` must define the method ``get_planning_scene``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``GetPlanningScene`` to be treated as its ``get_planning_scene`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, options=None):
        return self.get_planning_scene(options)

    @abstractmethod
    def get_planning_scene(self, options=None):
        """Retrieve the planning scene.

        Parameters
        ----------
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`compas_fab.robots.planning_scene.PlanningScene`
        """
        pass


class AddCollisionMesh(object):
    """Interface for a Planner's add collision mesh feature.  Any implementation of
    ``AddCollisionMesh`` must define the method ``add_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``AddCollisionMesh`` to be treated as its ``add_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, collision_mesh, options=None):
        return self.add_collision_mesh(collision_mesh, options)

    @abstractmethod
    def add_collision_mesh(self, collision_mesh, options=None):
        """Add a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be added.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass


class RemoveCollisionMesh(object):
    """Interface for a Planner's remove collision mesh feature.  Any implementation of
    ``RemoveCollisionMesh`` must define the method ``remove_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``RemoveCollisionMesh`` to be treated as its ``remove_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, id, options=None):
        return self.remove_collision_mesh(id, options)

    @abstractmethod
    def remove_collision_mesh(self, id, options=None):
        """Remove a collision mesh from the planning scene.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass


class AppendCollisionMesh(object):
    """Interface for a Planner's append collision mesh feature.  Any implementation of
    ``AppendCollisionMesh`` must define the method ``append_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``AppendCollisionMesh`` to be treated as its ``append_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, collision_mesh, options=None):
        return self.append_collision_mesh(collision_mesh, options)

    @abstractmethod
    def append_collision_mesh(self, collision_mesh, options=None):
        """Append a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be appended.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass


class AddAttachedCollisionMesh(object):
    """Interface for a Planner's add attached collision mesh feature.  Any implementation of
    ``AddAttachedCollisionMesh`` must define the method ``add_attached_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``AddAttachedCollisionMesh`` to be treated as its ``add_attached_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, attached_collision_mesh, options=None):
        return self.add_attached_collision_mesh(attached_collision_mesh, options)

    @abstractmethod
    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass


class RemoveAttachedCollisionMesh(object):
    """Interface for a Planner's remove attached collision mesh feature.  Any implementation of
    ``RemoveAttachedCollisionMesh`` must define the method ``remove_attached_collision_mesh``.  The
    ``__call__`` magic method allows an instance of an implementation of
    ``RemoveAttachedCollisionMesh`` to be treated as its ``remove_attached_collision_mesh`` method.  See
    <https://docs.python.org/3/reference/datamodel.html#object.__call__> and
    <https://en.wikipedia.org/wiki/Function_object#In_Python>.
    """
    __metaclass__ = ABCMeta

    def __call__(self, id, options=None):
        return self.remove_attached_collision_mesh(id, options)

    @abstractmethod
    def remove_attached_collision_mesh(self, id, options=None):
        """Remove an attached collision mesh from the robot.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass
