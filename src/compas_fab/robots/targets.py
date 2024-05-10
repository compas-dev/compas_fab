import compas

from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots.model import Joint

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Tuple  # noqa: F401
        from compas.geometry import Point  # noqa: F401
        from compas.geometry import Vector  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas_fab.robots import Constraint  # noqa: F401

__all__ = [
    "Target",
    "FrameTarget",
    "PointAxisTarget",
    "ConfigurationTarget",
    "ConstraintSetTarget",
]


class Target(Data):
    """Represents a kinematic target for motion planning.

    The current implementation supports only static target constraints such as
    pose, configuration, and joint constraints. Dynamic targets such as
    velocity, acceleration, and jerk are not yet supported.

    Targets are intended to be used as arguments for the Backend's motion
    planning methods. Different backends might support different types of
    targets.

    Attributes
    ----------
    name : str , optional, default = 'target'
        A human-readable name for identifying the target.

    See Also
    --------
    :class:`PointAxisTarget`
    :class:`FrameTarget`
    :class:`ConfigurationTarget`
    :class:`ConstraintSetTarget`
    """

    def __init__(self, name="Generic Target"):
        # type: (str) -> None
        super(Target, self).__init__()
        self.name = name

    @property
    def __data__(self):
        raise NotImplementedError

    def scaled(self, factor):
        # type: (float) -> Target
        """Returns a scaled copy of the target.

        Parameters
        ----------
        factor : float
            The scaling factor.

        Returns
        -------
        :class:`Target`
            The scaled target.
        """
        raise NotImplementedError


class FrameTarget(Target):
    """Represents a fully constrained pose target for the robot's end-effector using a :class:`compas.geometry.Frame`.

    When using a FrameTarget, the end-effector has no translational or rotational freedom.
    In another words, the pose of the end-effector is fully defined (constrained).

    Given a FrameTarget, the planner aims to find a path moving
    the robot's Tool0 Coordinate Frame (T0CF) to the specified `FrameTarget.target_frame`.

    If the user wants to plan with a tool (such that the Tool Coordinate Frame (TCF) is matched with the target),
    the `tool_coordinate_frame` parameter should be provided to define the TCF relative to the T0CF of the robot.

    For robots with multiple end effector attachment points (such as the RFL Robot), the attachment point depends on
    the planning group selected in the planning request, see :meth:`compas_fab.robots.Robot.plan_motion`.

    Attributes
    ----------
    target_frame : :class:`compas.geometry.Frame`
        The target frame.
    tolerance_position : float, optional
        The tolerance for the position.
        Unit is meters.
        If not specified, the default value from the planner is used.
    tolerance_orientation : float, optional
        The tolerance for the orientation.
        Unit is radians.
        If not specified, the default value from the planner is used.
    tool_coordinate_frame : :class:`compas.geometry.Frame` or :class:`compas.geometry.Transformation`, optional
        The tool tip coordinate frame relative to the flange of the robot.
        If not specified, the target frame is relative to the robot's flange.
    name : str, optional
        The name of the target.
        Defaults to 'Frame Target'.

    """

    def __init__(
        self,
        target_frame,
        tolerance_position=None,
        tolerance_orientation=None,
        tool_coordinate_frame=None,
        name="Frame Target",
    ):
        # type: (Frame, Optional[float], Optional[float], Optional[Frame | Transformation], Optional[str]) -> None
        super(FrameTarget, self).__init__(name=name)
        self.target_frame = target_frame
        self.tolerance_position = tolerance_position
        self.tolerance_orientation = tolerance_orientation
        if isinstance(tool_coordinate_frame, Transformation):
            tool_coordinate_frame = Frame.from_transformation(tool_coordinate_frame)
        self.tool_coordinate_frame = tool_coordinate_frame

    @property
    def __data__(self):
        return {
            "target_frame": self.target_frame,
            "tolerance_position": self.tolerance_position,
            "tolerance_orientation": self.tolerance_orientation,
            "tool_coordinate_frame": self.tool_coordinate_frame,
        }

    @classmethod
    def from_transformation(
        cls,
        transformation,
        tolerance_position=None,
        tolerance_orientation=None,
        tool_coordinate_frame=None,
        name="Frame Target",
    ):
        # type: (Transformation, Optional[float], Optional[float], Optional[Frame | Transformation], Optional[str]) -> FrameTarget
        """Creates a FrameTarget from a transformation matrix.

        Parameters
        ----------
        transformation : :class
            The transformation matrix.
        tolerance_position : float, optional
            The tolerance for the position.
            if not specified, the default value from the planner is used.
        tolerance_orientation : float, optional
            The tolerance for the orientation.
            if not specified, the default value from the planner is used.
        tool_coordinate_frame : :class:`compas.geometry.Frame` or :class:`compas.geometry.Transformation`, optional
            The tool tip coordinate frame relative to the flange of the robot.
            If not specified, the target frame is relative to the robot's flange.
        name : str, optional
            The name of the target.
            Defaults to 'Frame Target'.

        Returns
        -------
        :class:`FrameTarget`
            The frame target.
        """
        frame = Frame.from_transformation(transformation)
        return cls(frame, tolerance_position, tolerance_orientation, tool_coordinate_frame, name)

    def scaled(self, factor):
        # type: (float) -> FrameTarget
        """Returns a copy of the target where the target frame and tolerances are scaled.

        By convention, compas_fab robots use meters as the default unit of measure.
        If user model is created in millimeters, the FrameTarget should be scaled by a factor
        of 0.001 before passing to the planner.

        Parameters
        ----------
        factor : float
            The scaling factor.

        Returns
        -------
        :class:`FrameTarget`
            The scaled frame target.
        """
        target_frame = self.target_frame.scaled(factor)
        tolerance_position = self.tolerance_position * factor
        tolerance_orientation = self.tolerance_orientation * factor
        tool_coordinate_frame = self.tool_coordinate_frame.scaled(factor) if self.tool_coordinate_frame else None
        return FrameTarget(target_frame, tolerance_position, tolerance_orientation, tool_coordinate_frame, self.name)


class PointAxisTarget(Target):
    """
    Represents a point and axis target for the robot's end-effector motion planning.

    This target allows one degree of rotational freedom, enabling the end-effector
    to rotate around the target axis.
    Given a PointAxisTarget, the planner seeks a path to move the robot's end-effector
    tool tip to the target point and align the tool tip's Z axis with the specified target axis.

    PointAxisTarget is suitable for tasks like drilling, milling, and 3D printing,
    where aligning the end-effector with a target axis is crucial,
    but the orientation around the axis is not important.

    The user must define (1) the target point of which the tool tip will reach
    and (2) the target axis where the tool tip coordinate frame (TCF)'s Z axis
    can rotate around. The target point and axis are defined relative to the robot's
    world coordinate frame (WCF).

    In addition, it's necessary to define the tool tip coordinate frame (TCF)
    relative to the robot's flange frame. This is labeled as tool_coordinate_frame.
    If tool_coordinate_frame is unspecified, the target point and axis will be matched with the robot's flange frame.

    For robots with multiple end effector attachment points, the FCF depends on
    the planning group setting in the planning request, as defined in an SRDF file or
    :class:`compas_fab.robots.RobotSemantics`.

    Attributes
    ----------
    target_point : :class:`compas.geometry.Point`
        The target point defined relative to the world coordinate frame (WCF).
    target_z_axis : :class:`compas.geometry.Vector`
        The target axis is defined by the target_point and pointing towards this vector.
        The tool tip coordinate frame (TCF)'s Z axis can rotate around this axis.
    tolerance_position : float, optional
        The tolerance for the position of the target point.
        If not specified, the default value from the planner is used.
    tool_coordinate_frame : :class:`compas.geometry.Frame` or :class:`compas.geometry.Transformation`, optional
        The tool tip coordinate frame relative to the flange of the robot.
        If not specified, the target point is relative to the robot's flange (T0CF) and the
        Z axis of the flange can rotate around the target axis.
    name : str, optional
        The name of the target.
        Defaults to 'Point-Axis Target'.

    """

    def __init__(
        self,
        target_point,
        target_z_axis,
        tolerance_position=None,
        tool_coordinate_frame=None,
        name="Point-Axis Target",
    ):
        # type: (Point, Vector, Optional[float], Optional[Frame | Transformation], Optional[str]) -> None
        super(PointAxisTarget, self).__init__(name=name)
        self.target_point = target_point
        self.target_z_axis = target_z_axis
        self.tolerance_position = tolerance_position
        if isinstance(tool_coordinate_frame, Transformation):
            tool_coordinate_frame = Frame.from_transformation(tool_coordinate_frame)
        self.tool_coordinate_frame = tool_coordinate_frame

    def __data__(self):
        return {
            "target_point": self.target_point,
            "target_z_axis": self.target_z_axis,
            "tolerance_position": self.tolerance_position,
            "tool_coordinate_frame": self.tool_coordinate_frame,
        }

    def scaled(self, factor):
        # type: (float) -> PointAxisTarget
        """Returns a copy of the target where the target point and tolerances are scaled.

        Parameters
        ----------
        factor : float
            The scaling factor.

        Returns
        -------
        :class:`PointAxisTarget`
            The scaled point-axis target.
        """
        target_point = self.target_point.scaled(factor)
        tolerance_position = self.tolerance_position * factor if self.tolerance_position else None
        target_z_axis = self.target_z_axis.scaled  # Vector is unitized and is not scaled
        tool_coordinate_frame = self.tool_coordinate_frame.scaled(factor) if self.tool_coordinate_frame else None
        return PointAxisTarget(target_point, target_z_axis, tool_coordinate_frame, tolerance_position, self.name)


class ConfigurationTarget(Target):
    """Represents a configuration target for the robot's end-effector motion planning.

    The configuration target is a joint configuration of the robot's joints.
    Given a ConfigurationTarget, the planner aims to find a path moving
    the robot's joint positions to match with `ConfigurationTarget.target_configuration`.

    ConfigurationTarget is suitable for targets whose joint configuration is known,
    such as a home position, or a repetitive position that has been calibrated
    in the actual robot cell, such as a tool changing position.

    The number of joints in the target configuration should match the number of joints
    in the robot's planning group. Otherwise the behavior of the backend planner may
    be undefined. See tutorial :ref:`targets` for more details.

    Attributes
    ----------
    target_configuration : :class:`compas_robots.Configuration`
        The target configuration. joint_names and joint_values must be specified.
        Defaults unit is radians for revolute and continuous joints, and meters for prismatic joints.
    tolerance_above : :obj:`list` of :obj:`float`, optional
        Acceptable deviation above the targeted configurations. One for each joint.
        Always use positive values.
        If not specified, the default value from the planner is used.
    tolerance_below : :obj:`list` of :obj:`float`, optional
        Acceptable deviation below the targeted configurations. One for each joint.
        Always use positive values.
        If not specified, the default value from the planner is used.
    name : str, optional
        The name of the target.
        Defaults to 'Configuration Target'.
    """

    SUPPORTED_JOINT_TYPES = [Joint.PRISMATIC, Joint.REVOLUTE, Joint.CONTINUOUS]

    def __init__(self, target_configuration, tolerance_above=None, tolerance_below=None, name="Configuration Target"):
        # type: (Configuration, Optional[list[float]], Optional[list[float]], Optional[str]) -> None
        super(ConfigurationTarget, self).__init__(name=name)
        self.target_configuration = target_configuration  # type: Configuration
        self.tolerance_above = tolerance_above
        self.tolerance_below = tolerance_below

        # Check to make sure the joint types are supported
        for joint_type in target_configuration.joint_types:
            assert joint_type in self.SUPPORTED_JOINT_TYPES, "Unsupported joint type: {}".format(joint_type)

    def __data__(self):
        return {
            "target_configuration": self.target_configuration,
            "tolerance_above": self.tolerance_above,
            "tolerance_below": self.tolerance_below,
        }

    @classmethod
    def generate_default_tolerances(cls, configuration, tolerance_prismatic, tolerance_revolute):
        # type: (Configuration, float, float) -> Tuple[list[float], list[float]]
        """Generates tolerances values for the target configuration based on the joint types.

        The parameters `tolerance_prismatic` and `tolerance_revolute` are used to generate the
        list of values for `tolerances_above`, `tolerances_below`. The length of the list is equal to the
        number of joints in the target configuration.

        This function will not override the existing `tolerance_above` and `tolerance_below` values,
        users should set the values explicitly.

        Parameters
        ----------
        tolerance_prismatic : obj:`float`
            The default tolerance applied to prismatic joints.
            Defaults unit is meters.
        tolerance_revolute : obj:`float`
            The default tolerance applied to revolute and continuous joints.
            Defaults unit is radians.

        Returns
        -------
        :obj:`tuple` of (:obj:`list` of :obj:`float`, :obj:`list` of :obj:`float`)
            The tolerances_above and tolerances_below lists.

        Examples
        --------
        >>> from compas_robots import Configuration
        >>> from compas_fab.robots import ConfigurationTarget
        >>> from compas_robots.model import Joint
        >>> configuration = Configuration.from_revolute_values([0, 3.14, 0, 0, 3.14, 0])
        >>> tolerance_prismatic = 0.001
        >>> tolerance_revolute = math.radians(1)
        >>> tolerances_above, tolerances_below = ConfigurationTarget.generate_default_tolerances(configuration, tolerance_prismatic, tolerance_revolute)
        >>> target = ConfigurationTarget(configuration, tolerances_above, tolerances_below)

        """
        tolerances_above = []
        tolerances_below = []
        for joint_type in configuration.joint_types:
            if joint_type in [Joint.PRISMATIC]:
                tolerances_above.append(tolerance_prismatic)
                tolerances_below.append(tolerance_prismatic)
            elif joint_type in [Joint.REVOLUTE, Joint.CONTINUOUS]:
                tolerances_above.append(tolerance_revolute)
                tolerances_below.append(tolerance_revolute)
            else:
                raise NotImplementedError("Unsupported joint type: {}".format(joint_type))
        return tolerances_above, tolerances_below

    def scaled(self, factor):
        # type: (float) -> ConfigurationTarget
        """Returns copy of the target where the target configuration and tolerances are scaled.

        This function should only be needed if the ConfigurationTarget was created
        with a distance unit other than meters.

        Only the values for prismatic and planar joints are scaled. The values for revolute
        and continuous joints are not scaled, as they must be in radians.

        Parameters
        ----------
        factor : float
            The scaling factor.

        Returns
        -------
        :class:`ConfigurationTarget`
            The scaled configuration target.
        """
        target_configuration = self.target_configuration.scaled(factor)

        def scale_tolerance(tolerance, joint_types):
            # type: (list[float], list[Joint]) -> list[float]
            """Only scales the tolerances for prismatic and planar joints."""
            scaled_tolerance = []
            for t, joint_type in zip(tolerance, joint_types):
                if joint_type in (Joint.PLANAR, Joint.PRISMATIC):
                    t *= factor
                scaled_tolerance.append(t)
            return scaled_tolerance

        # We scale only the tolerances for prismatic and planar joints,
        # similar to the Configuration.scale() method
        tolerance_above = (
            scale_tolerance(self.tolerance_above, target_configuration.joint_types) if self.tolerance_above else None
        )
        tolerance_below = (
            scale_tolerance(self.tolerance_below, target_configuration.joint_types) if self.tolerance_below else None
        )

        return ConfigurationTarget(target_configuration, tolerance_above, tolerance_below, self.name)


class ConstraintSetTarget(Target):
    """Represents a list of Constraint as the target for motion planning.

    Given a ConstraintSetTarget, the planner aims to find a path moving
    the robot's end-effector to satisfy ALL the constraints in the constraint set.
    Constraints can be very specific, for example defining value domains
    for each joint, such that the goal configuration is included,
    or defining a volume in space, to which a specific robot link (e.g.
    the end-effector) is required to move to.

    This target cannot be translated into a single pose or configuration target
    except in trivial cases. Therefore the ending pose or configuration of the
    planned path can only be determined after performing the motion planning.

    ConstraintSetTarget is suitable for advanced users who want to specify
    custom constraints for the robot motion planning.
    Different planner backends may support different types of Constraints.
    See tutorial :ref:`targets` for more details.

    ConstraintSetTarget is only supported by Free motion planning,
    Cartesian motion planning do not support this target type.

    Attributes
    ----------
    constraint_set : :obj:`list` of :class:`compas_fab.robots.Constraint`
        A list of constraints to be satisfied.
    name : str, optional
        The name of the target.
        Defaults to 'Constraint Set Target'.
    """

    def __init__(self, constraint_set, name="Constraint Set Target"):
        # type: (list[Constraint], Optional[str]) -> None
        super(ConstraintSetTarget, self).__init__(name=name)
        self.constraint_set = constraint_set

    def __data__(self):
        return {"constraint_set": self.constraint_set}

    def scaled(self, factor):
        # type: (float) -> ConstraintSetTarget
        """Returns a scaled copy of the target.

        Raises
        ------
        NotImplementedError
            This target type does not support scaling.
        """
        raise NotImplementedError
