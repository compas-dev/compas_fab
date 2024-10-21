from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas_robots import Configuration

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas.geometry import Frame  # noqa: F401

        from compas_fab.robots import JointTrajectory  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from compas_fab.robots import TargetMode  # noqa: F401
        from compas_fab.robots import Waypoints  # noqa: F401


class BackendFeature(object):
    """Base class for all backend features that are implemented by a backend client.

    Classes that inherit from this class are mixed-in when creating the planner backend interface.
    Hence the the mixed-in class can access the attributes and other mix-ins functions of planner.

    The implemented feature classes can assume that `self` is an instance of the planner backend interface.
    IDE code completion and type hints can be activated by adding a line such as
    `planner = self  # type: MoveItPlanner`.

    Attributes
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`
        The backend client that supports this feature.


    """

    def __init__(self, client=None):
        super(BackendFeature, self).__init__()

    def _build_configuration(
        self, joint_positions, joint_names, group, return_full_configuration, start_configuration=None
    ):
        # type: (List[float], List[str], str, bool, Configuration) -> Configuration
        """This function helps the planner build a standard configuration that can be returned to the user.

        It supports two modes for different use cases:

        - If return_full_configuration is True, it returns a full configuration including all configurable
          joints in the robot, even if the joint is not in the planning group.
          See :meth:`compas_robots.model.Joint.is_configurable` for more details.
        - If return_full_configuration is False, it returns only the joint values for the planning group.
          The joint values are sorted according to the group's joint order.


        Parameters
        ----------
        joint_positions : list of float
            All joint values for the full configuration.
        joint_names : list of str
            The joint names corresponding to the joint values.
        group : str
            The name of the planning group.
        return_full_configuration : bool
            If True, the full configuration is returned, otherwise only the group configuration is returned.
        start_configuration : :class:`compas_robots.Configuration`, optional
            If return_full_configuration is True, this configuration is used to fill in the missing values.

        Notes
        -----
        Do not pass None to group when return_full_configuration is False, the behavior is undefined.

        """
        robot_cell = self.client.robot_cell  # type: RobotCell
        if return_full_configuration:
            # build configuration including passive joints, but no sorting
            configuration = robot_cell.zero_full_configuration()
            value_dict = dict(zip(joint_names, joint_positions))
            for name in configuration.joint_names:
                if name in value_dict:
                    configuration[name] = value_dict[name]
                elif start_configuration:
                    configuration[name] = start_configuration[name]
        else:
            # sort values for group configuration
            joint_state = dict(zip(joint_names, joint_positions))
            group_joint_names = robot_cell.get_configurable_joint_names(group)
            values = [joint_state[name] for name in group_joint_names]
            configuration = Configuration(values, robot_cell.get_configurable_joint_types(group), group_joint_names)

        return configuration


#   The code that contains the actual feature implementation is located in the backend's module.
#   For example, the features for moveit planner and ros client are located in :
#   "src/compas_fab/backends/ros/backend_features/"
#   Only in the case of `inverse_kinematics` feature, the implementation for managing repeated calls is provided in this file.
#   If you cannot find a specific feature in the 'backend_features', it means that the planner
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

    def set_robot_cell_state(self, robot_cell_state, options=None):
        # type: (RobotCellState, Optional[dict]) -> None
        """Set the robot cell state to the client.

        The client requires a robot cell state at the beginning of each planning request.
        This cell state must correspond to the robot cell set earlier by :meth:`set_robot_cell`.

        This function is called automatically by planning functions that takes a RobotCellState as input.
        Therefore it is typically not necessary for the user to call this function,
        except when used to trigger visualization using native backend canvas.

        """
        pass


class CheckCollision(BackendFeature):
    """Mix-in interface for implementing a planner's collision check feature."""

    def check_collision(self, robot_cell_state, options=None):
        # type: (RobotCellState, Optional[dict]) -> None
        """Check if the robot cell is in collision at the specified state.

        Different planners may have different criteria for collision checking, check the planner's documentation for details.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state to check for collision.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

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

    def forward_kinematics(self, robot_cell_state, target_mode, group=None, scale=None, options=None):
        # type: (RobotCellState, TargetMode | str, Optional[str], Optional[float], Optional[dict]) -> Frame
        """Calculate the target frame of the robot from the provided RobotCellState.

        The function can return the planner coordinate frame (PCF), the tool coordinate frame (TCF),
        or the workpiece's object coordinate frame (OCF) based on the ``target_mode`` provided.

        - ``"Target.ROBOT"`` will return the planner coordinate frame (PCF).
        - ``"Target.TOOL"`` will return the tool coordinate frame (TCF) if a tool is attached.
        - ``"Target.WORKPIECE"`` will return the workpiece's object coordinate frame (OCF)
          if a workpiece is attached.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration of the robot corresponding to the planning group.
            The Configuration object must include ``joint_names``.
            The robot cell state should also reflect the attachment of tools, if any.
        target_mode : :class:`compas_fab.robots.TargetMode` or str
            The target mode to select which frame to return.
        group : str, optional
            The planning group of the robot.
            Defaults to the robot's main planning group.
        scale : float, optional
            The scaling factor to apply to the resulting frame.
            For example, use ``'1000.0'`` to convert the result to millimeters.
            Defaults to None, which means no scaling is applied.
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

    def forward_kinematics_to_link(self, robot_cell_state, link_name=None, group=None, scale=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[str], Optional[float], Optional[dict]) -> Frame
        """Calculate the frame of the specified robot link from the provided RobotCellState.

        This function operates similar to :meth:`compas_fab.backends.PyBulletForwardKinematics.forward_kinematics`,
        but allows the user to specify which link to return. The function will return the frame of the specified
        link relative to the world coordinate frame (WCF).

        This can be convenient in scenarios where user objects (such as a camera) are attached to one of the
        robot's links and the user needs to know the position of the object relative to the world coordinate frame.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
        link_name : str, optional
            The name of the link to calculate the forward kinematics for.
            Defaults to the last link of the provided planning group.
        group : str, optional
            The planning group of the robot.
            Defaults to the robot's main planning group.
        scale : float, optional
            The scaling factor to apply to the resulting frame.
            For example, use ``'1000.0'`` to convert the result to millimeters.
            Defaults to None, which means no scaling is applied.
        options : dict, optional
            Dictionary for passing planner specific options.
            Currently unused.
        """


class InverseKinematics(BackendFeature):
    """Mix-in interface for implementing a planner's inverse kinematics feature."""

    def __init__(self):
        # The following fields are used to store the last ik request for iterative calls
        self._last_ik_request = {"request_hash": None, "solutions": None}

        # Initialize the super class
        super(InverseKinematics, self).__init__()

    def inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Configuration
        """Calculate the robot's inverse kinematic for a given frame.

        The default implementation is based on the iter_inverse_kinematics method.
        Calling this method will return the first solution found by the iterator,
        subsequent calls will return the next solution from the iterator. Once
        all solutions have been exhausted, the iterator will be re-initialized.

        The starting state describes the robot cell's state at the moment of the calculation.
        The robot's configuration is taken as the starting configuration.
        If a tool is attached to the planning group, the tool's coordinate frame is used.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.Target`
            The target to calculate the inverse kinematics for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
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
        :obj:`compas_robots.Configuration`
            The calculated configuration.

        """
        # This is the default implementation for the inverse kinematics feature to be based on the
        # iter_inverse_kinematics method. If the planner does not support this feature, it should
        # override this method and raise a BackendFeatureNotSupportedError.

        # The planner-specific implementation code is located in the backend's module:
        # "src/compas_fab/backends/<backend_name>/backend_features/<planner_name>_inverse_kinematics"

        # Pseudo-memoized sequential calls will re-use iterator if not exhausted
        request_hash = (target.sha256(), robot_cell_state.sha256(), str(group), str(options))

        if self._last_ik_request["request_hash"] == request_hash and self._last_ik_request["solutions"] is not None:
            solution = next(self._last_ik_request["solutions"], None)
            # NOTE: If the iterator is exhausted, solution will be None, subsequent code outside will reset the generator
            if solution is not None:
                return solution

        solutions = self.iter_inverse_kinematics(target, robot_cell_state, group, options)
        self._last_ik_request["request_hash"] = request_hash
        self._last_ik_request["solutions"] = solutions

        # NOTE: If the 'solutions' generator cannot yield even one solution, it will raise an exception here:
        return next(solutions)

    def iter_inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Configuration
        """Calculate the robot's inverse kinematic for a given frame.

        This function returns a generator that yields possible solutions for the
        inverse kinematics problem. The generator will be exhausted after all
        possible solutions have been returned or when the maximum number of
        results has been reached.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.Target`
            The target to calculate the inverse kinematics for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
            The starting state to calculate the inverse kinematics for.
            The robot's configuration in the scene is taken as the starting configuration.
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
        :class:`compas_robots.Configuration`
            The calculated configuration.

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

    def plan_cartesian_motion(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
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
