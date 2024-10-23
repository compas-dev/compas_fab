from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas.geometry import Frame  # noqa: F401


class AnalyticalKinematics(object):
    """Base class for analytical kinematics solvers.

    This class is meant to be subclassed by kinematics solvers specific to a robot.
    Note that robot models in compas comply to URDF standards, which allows for
    arbitrary robot chains, however this is not the case in analytical kinematics.
    Even for 6R robots, whose kinematic chain that can be represented by a DH table
    used for analytical kinematics, the base and flange frames can differ from the URDF model.
    Individual joint values may also differ from the one defined by URDF by a constant offset.

    In order to match the behavior of the analytical kinematics solver with planners that
    uses the URDF model, the AnalyticalKinematics class provides the attributes to


    Attributes
    ----------
    base_frame_offset : :class:`compas.geometry.Frame`, Optional
        The base frame of the analytical robot relative to the world coordinate system.
        This should be the same as the base frame of the robot in the URDF model.
        Defaults to no offset.
    flange_frame_offset : :class:`compas.geometry.Frame`, Optional
        The end-effector frame of the robot relative to the last joint of the analytical robot.
        This should match the end effector of the robot in the URDF model.
        Defaults to no offset.
    joint_offsets : list of float, Optional
        The offset to be added to each joint value of the analytical robot to match the URDF model.
        ``J_robotmodel = J_analytical + joint_offsets``
        Defaults to no offset.
    """

    def __init__(self, base_frame=None, flange_frame=None, joint_offsets=None):
        # type: (Optional[Frame], Optional[Frame], Optional[List[float]]) -> None
        super(AnalyticalKinematics, self).__init__()
        self.base_frame = base_frame
        self.flange_frame = flange_frame
        self.joint_offsets = joint_offsets

    def forward(self, joint_values):
        # type: (List[float]) -> Frame
        """Calculate the forward kinematics for the given joint configuration.

        Parameters
        ----------
        joint_values : list of float
            The joint configuration values.
            Joint values must be in radians.

        Returns
        -------
        :class:`compas.geometry.Frame`
            The frame of the end-effector.
        """
        pass

    def inverse(self, frame_rcf):
        # type: (Frame) -> List[List[float]]
        """Calculate the inverse kinematics for the given end-effector frame.

        Parameters
        ----------
        frame_rcf : :class:`compas.geometry.Frame`
            The frame of the end-effector.

        Returns
        -------
        list of list of float
            A list of all possible IK solutions.
            Joint values are in radians.
        """
        pass
