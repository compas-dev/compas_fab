from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.geometry import Frame
from compas.geometry import Transformation
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

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Optional  # noqa: F401

        from compas_fab.backends.ros.client import RosClient  # noqa: F401
        from compas_fab.backends.ros.planner import MoveItPlanner  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import TargetMode  # noqa: F401


__all__ = [
    "MoveItForwardKinematics",
]


class MoveItForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""

    GET_POSITION_FK = ServiceDescription(
        "/compute_fk", "GetPositionFK", GetPositionFKRequest, GetPositionFKResponse, validate_response
    )

    def forward_kinematics(self, robot_cell_state, target_mode, group=None, native_scale=None, options=None):
        # type: (RobotCellState, TargetMode, Optional[str], Optional[float], Optional[dict]) -> Frame
        """Calculate the target frame of the robot (relative to WCF) from the provided RobotCellState.

        The returned coordinate frame is dependent on the chosen ``target_mode``:

        - ``"Target.ROBOT"`` will return the planner coordinate frame (PCF).
        - ``"Target.TOOL"`` will return the tool coordinate frame (TCF) if a tool is attached.
        - ``"Target.WORKPIECE"`` will return the workpiece's object coordinate frame (OCF)
          if a workpiece is attached (via an attached tool).

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration of the robot corresponding to the planning group.
            The robot cell state should also reflect the attachment of tools, if any.
        target_mode : :class:`compas_fab.robots.TargetMode` or str
            The target mode to select which frame to return.
        group : str, optional
            The planning group of the robot.
            Defaults to the robot's main planning group.
        native_scale : float, optional
            The scaling factor to apply to the resulting frame.
            It is defined as `user_object_value * native_scale = meter_object_value`.
            For example, if the resulting frame is to be used in a millimeters environment, `native_scale` should be set to ``'0.001'``.
            Defaults to None, which means no scaling is applied.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the model's root link.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell
        options = options or {}

        group = group or robot_cell.main_group_name

        link_name = robot_cell.get_end_effector_link_name(group)
        pcf_frame = self.forward_kinematics_to_link(robot_cell_state, link_name, native_scale, options)

        # Convert PCF to the target frame
        target_frame = client.robot_cell.pcf_to_target_frames(robot_cell_state, pcf_frame, target_mode, group)
        t_rcf_target = Transformation.from_frame(target_frame)

        # Convert target frame to WCF
        t_wcf_rcf = Transformation.from_frame(robot_cell_state.robot_base_frame)
        t_wcf_target = t_wcf_rcf * t_rcf_target
        wcf_target_frame = Frame.from_transformation(t_wcf_target)

        # Scale resulting frame to user units
        if native_scale:
            wcf_target_frame.scale(1 / native_scale)

        return wcf_target_frame

        return target_frame

    def forward_kinematics_to_link(self, robot_cell_state, link_name=None, native_scale=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[float], Optional[dict]) -> Frame
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
        native_scale : float, optional
            The scaling factor to apply to the resulting frame.
            It is defined as `user_object_value * native_scale = meter_object_value`.
            For example, if the resulting frame is to be used in a millimeters environment, `native_scale` should be set to ``'0.001'``.
            Defaults to None, which means no scaling is applied.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the model's root link.
        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell
        options = options or {}

        if robot_cell.robot_model.get_link_by_name(link_name) is None:
            raise ValueError("Link name {} does not exist in robot model".format(link_name))

        planner.set_robot_cell_state(robot_cell_state)

        # Compose the kwargs for the forward kinematics
        kwargs = {}
        kwargs["configuration"] = robot_cell_state.robot_configuration
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"
        options["base_link"] = options.get("base_link", robot_cell.root_name)
        options["link"] = link_name

        lcf_frame = await_callback(self.forward_kinematics_async, **kwargs)
        return lcf_frame

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
