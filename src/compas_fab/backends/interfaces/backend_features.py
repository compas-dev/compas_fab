from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import FrameTarget  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401


class BackendFeature(object):
    """Base class for all backend features that are implemented by a backend client.

    Attributes
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`
        The backend client that supports this feature.

    """

    def __init__(self, client=None):
        super(BackendFeature, self).__init__()

    def _scale_input_target(self, target):
        # type: (Target) -> Target
        """Scale the target if the robot has user defined scale."""
        robot = self.robot_cell.robot  # type: Robot

        # Check `robot.need_scaling` to avoid unnecessary scaling
        if robot.need_scaling:
            # Scale input target back to meter scale
            return target.scaled(1.0 / robot.scale_factor)

        return target

    def _scale_output_frame(self, frame):
        # type: (Frame) -> Frame
        """Scale the frame if the robot has user defined scale."""
        robot = self.robot_cell.robot  # type: Robot

        # Check `robot.need_scaling` to avoid unnecessary scaling
        if robot.need_scaling:
            # Scale output frame to user defined scale
            return frame.scaled(robot.scale_factor)

        return frame

    # @property
    # def client(self):
    #     # type: () -> ClientInterface
    #     """Proxy function to access the backend client.
    #     This function should be overridden by the PlannerInterface default :meth:`PlannerInterface.client` or by the Planner.
    #     """
    #     raise NotImplementedError

    # @property
    # def robot_cell(self):
    #     # type: () -> RobotCell
    #     """Proxy function to access the RobotCell object.
    #     This function should be overridden by the PlannerInterface default :meth:`PlannerInterface.robot_cell` or by the Planner.
    #     """
    #     raise NotImplementedError


#   The code that contains the actual feature implementation is located in the backend's module.
#   For example, the features for moveit planner and ros client are located in :
#   "src/compas_fab/backends/ros/backend_features/"
#   If you cannot a specific feature in the 'backend_features', it means that the planner
#   does not support that feature.


class SetRobotCell(BackendFeature):
    """Mix-in interface for implementing a planner's set robot cell feature."""

    def set_robot_cell(self, robot_cell, robot_cell_state=None, options=None):
        # type: (RobotCell, Optional[RobotCellState], Optional[Dict]) -> None
        """Pass the models in the robot cell to the planning client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        """
        pass


class SetRobotCellState(BackendFeature):
    """Mix-in interface for implementing a planner's set robot cell state feature."""

    def set_robot_cell_state(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """Set the robot cell state to the client.

        The client requires a robot cell state at the beginning of each planning request.
        This cell state must correspond to the robot cell set earlier by :meth:`set_robot_cell`.

        This function is called automatically by planning functions that takes a start_state as input.
        Therefore it is typically not necessary for the user to call this function,
        except when used to trigger visualization using native backend canvas.

        """
        pass


class CheckCollision(BackendFeature):
    """Mix-in interface for implementing a planner's collision check feature."""

    def check_collision(self, robot_cell_state, allowed_collision_pairs=[]):
        # type: (RobotCellState, List[Tuple[str,str]]) -> None
        """Check if the robot cell is in collision at the specified state.


        Collision checking typically involves checking for collisions between
        (1) the robot's links, a.k.a. self-collision;
        (2) the robot's links and the attached objects;
        (3) the robot's links and stationary objects;
        (4) the attached objects and the stationary objects.
        (5) the attached objects and other attached objects.
        Where the attached and stationary objects refers to environment rigid bodies, tools and workpieces.

        Different planners may have different criteria for collision checking, check the planner's documentation for details.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state to check for collision.
        allowed_collision_pairs : :obj:`list` of :obj:`tuple` of (str, str)
            Collision pairs that are allowed to collide.
            The pairs are defined by the names of the collision objects.
            - For robot links, the names are the link names of the robot prefixed by the robot's name.
              For example, a robot named "ur5" with a link named "link" will have the name "robot/link".


        Returns
        -------
        None
            If there is collision, the function will raise exception, otherwise it will return None.

        Raises
        ------
        :class:`compas_fab.backends.exceptions.CollisionCheckError`
            If collision is detected.

        """
        pass


class ForwardKinematics(BackendFeature):
    """Mix-in interface for implementing a planner's forward kinematics feature."""

    def forward_kinematics(self, robot_cell_state, group=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[Dict]) -> Frame
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which forward kinematics is being calculated.
        configuration : :class:`compas_robots.Configuration`
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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_forward_kinematics.py"


class InverseKinematics(BackendFeature):
    """Mix-in interface for implementing a planner's inverse kinematics feature."""

    def __init__(self):
        # The following fields are used to store the last ik request for iterative calls
        self._last_ik_request = {"request_hash": None, "solutions": None}

        # Initialize the super class
        super(InverseKinematics, self).__init__()

    def inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Tuple[List[float], List[str]]
        """Calculate the robot's inverse kinematic for a given frame.

        The default implementation is based on the iter_inverse_kinematics method.
        Calling this method will return the first solution found by the iterator,
        subsequent calls will return the next solution from the iterator. Once
        all solutions have been exhausted, the iterator will be re-initialized.

        The starting state describes the robot cell's state at the moment of the calculation.
        The robot's configuration is taken as the starting configuration.
        If a tool is attached to the planning group, the tool's coordinate frame is used.

        If the backend supports collision checking,

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.FrameTarget`
            The frame target to calculate the inverse kinematics for.
        start_state : :class:`compas_fab.robots.RobotCellState`, optional
            The starting state to calculate the inverse kinematics for.
            The robot's configuration in the scene is taken as the starting configuration.
        group : str, optional
            The planning group used for calculation.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.
            - ``"return_full_configuration"``: (:obj:`bool`) If ``True``, the full configuration
                will be returned. Defaults to ``False``.
            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
                Defaults to ``100``.

        Raises
        ------
        :class: `compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        Returns
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.
        """
        # This is the default implementation for the inverse kinematics feature to be based on the
        # iter_inverse_kinematics method. If the planner does not support this feature, it should
        # override this method and raise a BackendFeatureNotSupportedError.

        # The planner-specific implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_inverse_kinematics"

        # Pseudo-memoized sequential calls will re-use iterator if not exhausted
        request_hash = (target.sha256(), start_state.sha256(), str(group), str(options))

        if self._last_ik_request["request_hash"] == request_hash and self._last_ik_request["solutions"] is not None:
            solution = next(self._last_ik_request["solutions"], None)
            if solution is not None:
                return solution

        solutions = self.iter_inverse_kinematics(target, start_state, group, options)
        self._last_ik_request["request_hash"] = request_hash
        self._last_ik_request["solutions"] = solutions

        return next(solutions)

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Tuple[List[float], List[str]]
        """Calculate the robot's inverse kinematic for a given frame.

        This function returns a generator that yields possible solutions for the
        inverse kinematics problem. The generator will be exhausted after all
        possible solutions have been returned or when the maximum number of
        results has been reached.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing planner-specific arguments.
            See the planner's documentation for supported options.
            One of the supported options related to the iterator is:

            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
              Defaults to ``100``.

        Raises
        ------
        compas_fab.backends.exceptions.InverseKinematicsError
            If no configuration can be found.

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
        target : :class:`compas_fab.robots.Target`
            The goal for the robot to achieve.
        start_configuration : :class:`compas_robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group : str, optional
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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_plan_motion.py"


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
        start_configuration : :class:`compas_robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group : str, optional
            The planning group used for calculation.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        pass

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_plan_cartesian_motion.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_get_planning_scene.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_reset_planning_scene.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_add_collision_mesh.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_append_collision_mesh.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_add_attached_collision_mesh.py"


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

        # The implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_remove_attached_collision_mesh.py"
