from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401


class BackendFeature(object):
    """Base class for all backend features that are implemented by a backend client."""

    def __init__(self, client):
        # All backend features are assumed to be associated with a backend client.
        self.client = client  # type: ClientInterface


#   The code that contains the actual feature implementation is located in the backend's module.
#   For example, the features for moveit planner and ros client are located in :
#   "src/compas_fab/backends/ros/backend_features/"
#   If you cannot a specific feature in the 'backend_features', it means that the planner
#   does not support that feature.


class ForwardKinematics(BackendFeature):
    """Mix-in interface for implementing a planner's forward kinematics feature."""

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


class InverseKinematics(BackendFeature):
    """Mix-in interface for implementing a planner's inverse kinematics feature."""

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Note that unlike other backend features, `inverse_kinematics` produces a generator.

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

        Yields
        ------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.
        """
        pass


class PlanMotion(BackendFeature):
    """Mix-in interface for implementing a planner's plan motion feature."""

    def plan_motion(self, robot, target, start_configuration=None, group=None, options=None):
        """Calculates a motion path.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion path is being calculated.
        target: :class:`compas_fab.robots.Target`
            The goal for the robot to achieve.
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


class PlanCartesianMotion(BackendFeature):
    """Mix-in interface for implementing a planner's plan cartesian motion feature."""

    def plan_cartesian_motion(self, robot, waypoints, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion path is being calculated.
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_configuration: :class:`compas_robots.Configuration`, optional
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


class GetPlanningScene(BackendFeature):
    """Mix-in interface for implementing a planner's get planning scene feature."""

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


class ResetPlanningScene(BackendFeature):
    """Mix-in interface for implementing a planner's reset planning scene feature."""

    def reset_planning_scene(self, options=None):
        """Resets the planning scene, removing all added collision meshes.

        Parameters
        ----------
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        ``None``
        """
        pass


class AddCollisionMesh(BackendFeature):
    """Mix-in interface for implementing a planner's add collision mesh feature."""

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


class RemoveCollisionMesh(BackendFeature):
    """Mix-in interface for implementing a planner's remove collision mesh feature."""

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


class AppendCollisionMesh(BackendFeature):
    """Mix-in interface for implementing a planner's append collision mesh feature."""

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


class AddAttachedCollisionMesh(BackendFeature):
    """Mix-in interface for implementing a planner's add attached collision mesh feature."""

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


class RemoveAttachedCollisionMesh(BackendFeature):
    """Mix-in interface for implementing a planner's remove attached collision mesh feature."""

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
