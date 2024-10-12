from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import GetPositionFKRequest
from compas_fab.backends.ros.messages import GetPositionFKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.service_description import ServiceDescription

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends.ros.client import RosClient  # noqa: F401
        from compas_fab.backends.ros.planner import MoveItPlanner  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import TargetMode  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas.geometry import Frame  # noqa: F401

__all__ = [
    "MoveItForwardKinematics",
]


class MoveItForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""

    GET_POSITION_FK = ServiceDescription(
        "/compute_fk", "GetPositionFK", GetPositionFKRequest, GetPositionFKResponse, validate_response
    )

    def forward_kinematics(self, robot_cell_state, target_mode, group=None, options=None):
        # type: (RobotCellState, TargetMode, str, dict) -> Frame
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state.
        target_mode : :class:`compas_fab.robots.TargetMode`
            The target mode.
        group : str, optional
            Unused parameter.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the model's root link.
            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the group's end
              effector link.
              Backwards compatibility note: if there's no ``link`` option, the
              planner will try also ``tool0`` as fallback before defaulting
              to the end effector's link.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot = client.robot  # type: Robot
        options = options or {}

        planner.set_robot_cell_state(robot_cell_state)

        kwargs = {}
        kwargs["configuration"] = robot_cell_state.robot_configuration
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot.model.root.name)

        group = group or robot.main_group_name

        # Use selected link or default to group's end effector
        options["link"] = options.get("link", options.get("tool0")) or robot.get_end_effector_link_name(group)
        if options["link"] not in robot.get_link_names(group):
            raise ValueError("Link name {} does not exist in planning group".format(options["link"]))

        pcf_frame = await_callback(self.forward_kinematics_async, **kwargs)
        return client.robot_cell.pcf_to_target_frames(robot_cell_state, pcf_frame, target_mode, group)

    def forward_kinematics_async(self, callback, errback, configuration, options):
        """Asynchronous handler of MoveIt FK service."""
        base_link = options["base_link"]
        fk_link_names = [options["link"]]

        header = Header(frame_id=base_link)
        joint_state = JointState(name=configuration.joint_names, position=configuration.joint_values, header=header)
        robot_state = RobotState(joint_state, MultiDOFJointState(header=header))
        robot_state.filter_fields_for_distro(self.client.ros_distro)

        def convert_to_frame(response):
            callback(response.pose_stamped[0].pose.frame)

        self.GET_POSITION_FK(self.client, (header, fk_link_names, robot_state), convert_to_frame, errback)
