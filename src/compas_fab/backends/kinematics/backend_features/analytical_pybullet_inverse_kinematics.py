import compas

from .analytical_inverse_kinematics import AnalyticalInverseKinematics
from compas_fab.backends.exceptions import BackendTargetNotSupportedError

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import AnalyticalPyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import Dict, List, Optional, Tuple  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401

from compas_robots import Configuration

from compas_fab.backends import CollisionCheckError
from compas_fab.robots import FrameTarget

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

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given target.

        An iterator is returned that yields configurations.


        """

        group = group or self.client.robot.main_group_name

        if isinstance(target, FrameTarget):
            return self.iter_inverse_kinematics_frame_target(
                target, start_state=start_state, group=group, options=options
            )
        else:
            raise BackendTargetNotSupportedError()

    def iter_inverse_kinematics_frame_target(self, target, start_state, group, options=None):
        # type: (FrameTarget, RobotCellState, str, Optional[Dict]) -> Generator[Configuration | None]
        """This function overrides the iter_inverse_kinematics_frame_target function from AnalyticalInverseKinematics
        to include the PyBulletClient for collision checking.

        It depends on the PyBulletClient to be able to check for collisions.

        """
        # TODO: This function is migrated but not finished with new CC functions.

        options = options or {}
        planner = self  # type: AnalyticalPyBulletPlanner
        robot = planner.client.robot_cell.robot  # type: Robot
        client = self.client  # type: PyBulletClient

        # Set robot cell state to start state if provided
        planner.set_robot_cell_state(start_state)

        # Scale Target and get target frame
        target = target.normalized_to_meters()
        target_frame = target.target_frame

        # Tool Coordinate Frame if there are tools attached
        target_pcf_frame = planner.target_frames_to_pcf(target_frame, target.target_mode, group)

        ik_configurations = self.inverse_kinematics_ordered(target_pcf_frame, group=group, options=options)
        for configuration in ik_configurations:
            # NOTE: keep_order parameter is handled in the inverse_kinematics_ordered function

            # Directly yield None, None if the solution is None, None
            if configuration is None:
                yield configuration
                continue

            # Check for collisions, skip yielding the solution if CC fails
            if options.get("check_collision", True):
                try:
                    # To save some time, we only set the robot configuration but avoid calling set_robot_cell_states
                    client.set_robot_configuration(configuration)
                    # Passing the `_skip_set_robot_cell_state` option to the collision check function
                    planner.check_collision(None, options={"_skip_set_robot_cell_state": True})
                except CollisionCheckError as e:
                    # If keep order is true, yield a None to keep the order of the solutions
                    if options.get("keep_order", True):
                        yield None
                    # If keep order is false, skip the solution
                    else:
                        continue
            else:
                # Yield the configuration without testing CC
                yield configuration
