from compas import IPY

from compas_fab.backends.exceptions import BackendTargetNotSupportedError

from .analytical_inverse_kinematics import AnalyticalInverseKinematics

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from compas_fab.backends import AnalyticalPyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from typing import Generator  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import Optional  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401

from compas_fab.backends import CollisionCheckError
from compas_fab.robots import FrameTarget


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
        client = self.client  # type: PyBulletClient

        group = group or client.robot_cell.main_group_name

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
        client = self.client  # type: PyBulletClient

        # Set robot cell state to start state if provided
        planner.set_robot_cell_state(start_state)

        # Scale Target and get target frame
        target = target.normalized_to_meters()
        target_frame = target.target_frame

        # Tool Coordinate Frame if there are tools attached
        target_pcf_frame = client.robot_cell.target_frames_to_pcf(start_state, target_frame, target.target_mode, group)

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
                    client._set_robot_configuration(configuration)
                    # Passing the `_skip_set_robot_cell_state` option to the collision check function
                    planner.check_collision(None, options={"_skip_set_robot_cell_state": True})
                except CollisionCheckError:
                    # If keep order is true, yield a None to keep the order of the solutions
                    if options.get("keep_order", True):
                        yield None
                    # If keep order is false, skip the solution
                    else:
                        continue
            else:
                # Yield the configuration without testing CC
                yield configuration
