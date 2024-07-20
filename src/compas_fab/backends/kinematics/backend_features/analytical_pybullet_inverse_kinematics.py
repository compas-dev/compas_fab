from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.exceptions import BackendTargetNotSupportedError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from .analytical_inverse_kinematics import AnalyticalInverseKinematics
import compas


if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import AnalyticalKinematicsPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import Dict, List, Optional, Tuple  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401


from compas_fab.robots import FrameTarget
from compas_robots import Configuration


from ..utils import joint_angles_to_configurations
from ..utils import try_to_fit_configurations_between_bounds


class AnalyticalPybulletInverseKinematics(AnalyticalInverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame.

    Includes the ability to use PyBullet for collision checking.

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

    def iter_inverse_kinematics_frame_target(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Tuple[List[float], List[str]], None, None]
        """This function overrides the iter_inverse_kinematics_frame_target function from AnalyticalInverseKinematics
        to include the PyBulletClient for collision checking.

        It depends on the PyBulletClient to be able to check for collisions.

        """
        options = options or {}
        planner = self  # type: AnalyticalKinematicsPlanner
        robot = planner.robot_cell.robot  # type: Robot
        client = self.client  # type: PyBulletClient

        # Scale Target and get target frame
        target = self._scale_input_target(target)
        target_frame = target.target_frame

        # Tool Coordinate Frame if there are tools attached
        attached_tool_id = start_state.get_attached_tool_id(group)
        if attached_tool_id:
            target_frame = self.from_tcf_to_t0cf([target_frame], attached_tool_id)[0]

        solutions = self.inverse_kinematics_ordered(target_frame, group=group, options=options)
        for solution in solutions:
            # Directly yield None, None if the solution is None, None
            if solution == (None, None):
                yield solution
                continue

            # Check for collisions, skip yielding the solution if CC fails
            if options.get("check_collision", False):
                try:
                    joint_values, joint_names = solution
                    client.check_collisions(robot, Configuration.from_revolute_values(joint_values, joint_names))
                except BackendError as e:
                    # If keep order is true, yield (None, None) to keep the order of the solutions
                    if options.get("keep_order", False):
                        yield (None, None)
                    continue

            yield solution
