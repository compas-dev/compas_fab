from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.exceptions import BackendTargetNotSupportedError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics

import compas


if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas.geometry import Frame  # noqa: F401
        from compas_fab.backends import AnalyticalKinematicsPlanner  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from typing import Dict, List, Optional, Tuple  # noqa: F401
        from typing import Generator  # noqa: F401

from ..utils import joint_angles_to_configurations
from ..utils import try_to_fit_configurations_between_bounds

from compas_fab.robots import FrameTarget


class AnalyticalInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame.

    Parameters
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, optional
        The backend client to use for communication.
    solver : :obj:`str`, optional
        The solver to use to calculate IK.

    Notes
    -----
    This works only for industrial robot arms with six revolute joints.
    If ``check_collision`` is `True`, it is required to use a client
    that supports ``"check_collision"``, so for now only the `PyBulletClient`.
    """

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given target.

        An iterator is returned that yields configurations.


        """
        if isinstance(target, FrameTarget):
            return self.iter_inverse_kinematics_frame_target(
                target, start_state=start_state, group=group, options=options
            )
        else:
            raise BackendTargetNotSupportedError()

    def iter_inverse_kinematics_frame_target(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given frame target.

        The IK for 6-axis industrial robots returns by default 8 possible solutions.
        These solutions have an order. That means that if you call IK on two
        subsequent frames and compare the 8 configurations of the first frame
        with the 8 configurations of the second frame at their respective indices,
        then these configurations are "close" to one another. For this reason,
        for certain use cases, e.g. for cartesian path planning, it makes sense
        to keep the order of solutions. This can be achieved by setting the
        optional parameter ``keep_order`` to ``True``. The configurations that
        are in collision or outside joint boundaries are then not removed from
        the list of solutions, they are set to ``None``.



        This function handles the case where the target is a :class:`compas_fab.robots.FrameTarget`.
        """
        planner = self  # type: AnalyticalKinematicsPlanner

        # Scale Target and get target frame
        target = target.normalized_to_meters()
        target_frame = target.target_frame

        target_frame = planner.target_frames_to_pcf(target_frame, target.target_mode, group=group)

        return self.inverse_kinematics_ordered(target_frame, group=group, options=options)

    def inverse_kinematics_ordered(self, frame_WCF, group=None, options=None):
        # type: (Frame, Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic (IK) for a given frame.

        The IK for 6-axis industrial robots returns by default 8 possible solutions.
        These solutions have an order. That means that if you call IK on two
        subsequent frames and compare the 8 configurations of the first frame
        with the 8 configurations of the second frame at their respective indices,
        then these configurations are "close" to one another. For this reason,
        for certain use cases, e.g. for cartesian path planning, it makes sense
        to keep the order of solutions. This can be achieved by setting the
        optional parameter ``keep_order`` to ``True``. The configurations that
        are in collision or outside joint boundaries are then not removed from
        the list of solutions, they are set to ``None``.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        group: str, optional
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict
            Dictionary containing the following key-value pairs:

            - ``"solver"``: (:obj:`str`) The solver to use to calculate IK.
            - ``"check_collision"``: (:obj:`str`, optional ) When ``True``, checks
              if the robot is in collision. Defaults to ``False``.
            - ``"keep_order"``: (:obj:`str`, optional ) When ``False``, removes the
              ``None``- solutions. Defaults to ``False``.

        Yields
        ------
        :obj:`compas_robots.Configuration`
            The calculated configuration.
            If ``"keep_order"`` is ``True`` result may contain ``None``.

        Notes
        -----
        This will only work with robots that have 6 revolute joints.
        This function will yield exactly 8 times, if ``"keep_order"`` is ``True``.

        Raises
        ------
        ValueError
            If the solver to solve the kinematics has not been passed.
        InverseKinematicsError
            If no IK solution could be found by the kinematic solver.
        """

        options = options or {}
        planner = self  # type: AnalyticalKinematicsPlanner
        robot = planner.client.robot_cell.robot  # type: Robot
        solver = planner.kinematics_solver

        keep_order = options.get("keep_order", False)

        # convert the frame WCF to RCF
        if solver.base_frame is not None:
            frame_RCF = solver.base_frame.to_local_coordinates(frame_WCF)
        else:
            frame_RCF = frame_WCF

        # calculate inverse with 8 solutions
        try:
            solutions = solver.inverse(frame_RCF)
        except ValueError:
            raise InverseKinematicsError()
        configurations = joint_angles_to_configurations(robot, solutions, group=group)

        # fit configurations within joint bounds (>> sets those to `None` that are not working)
        configurations = try_to_fit_configurations_between_bounds(robot, configurations, group=group)

        if not any(configurations):
            raise InverseKinematicsError()

        for config in configurations:
            # if keep_order is False, remove None solutions
            if not keep_order and config is None:
                continue
            # otherwise yield also the None solutions
            yield config
