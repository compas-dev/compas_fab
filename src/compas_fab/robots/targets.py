from copy import deepcopy

from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.tolerance import TOL
from compas_robots.model import Joint

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas.geometry import Transformation  # noqa: F401
        from compas_robots import Configuration  # noqa: F401

        from compas_fab.robots import Constraint  # noqa: F401

__all__ = [
    "ConfigurationTarget",
    "ConstraintSetTarget",
    "FrameTarget",
    "FrameWaypoints",
    "PointAxisTarget",
    "PointAxisWaypoints",
    "Target",
    "TargetMode",
    "Waypoints",
]


class Target(Data):
    """Represents a kinematic target for motion planning.

    The current implementation supports only static target constraints such as
    pose, configuration, and joint constraints. Dynamic targets such as
    velocity, acceleration, and jerk are not yet supported.

    Targets are intended to be used for motion planning with a planning backend
    by using :meth:`compas_fab.robot.plan_motion`.
    Note that different backends support different types of targets.

    Attributes
    ----------
    name : str , optional, default = 'target'
        A human-readable name for identifying the target.
    target_mode : :class:`TargetMode` or str, optional
        The target mode specifies which link or frame is referenced when specifying a target.
        This attribute is optional in this base class because some child
        classes (e.g: ConfigurationTarget) do not require it.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.

    See Also
    --------
    :class:`PointAxisTarget`
    :class:`FrameTarget`
    :class:`ConfigurationTarget`
    :class:`ConstraintSetTarget`
    """

    def __init__(self, target_mode=None, target_scale=1.0, name="Generic Target"):
        # type: (TargetMode|str, float, str) -> None
        super(Target, self).__init__()
        self.name = name
        self.target_mode = target_mode
        self.target_scale = target_scale

    @property
    def __data__(self):
        raise NotImplementedError

    def normalize_to_meters(self):
        # type: () -> None
        """Convert the target into meter scale if `target_scale` is not 1.0.

        Because all robots and planners in compas_fab use meters as the default unit of measure,
        user targets that are created in other units (e.g. millimeters) will have a `target_scale`
        factor such as 0.001 for millimeters.

        This function convert the target and its tolerance into meters scale
        and setting the `target_scale` attribute to 1.0.

        Notes
        -----

        This function will modify the target in place.
        It can only be called once because the target will be in meters scale after the first call.
        """
        raise NotImplementedError

    def normalized_to_meters(
        self,
    ):
        # type: () -> Target
        """Returns a copy of the target where the target and tolerances are scaled to meters.

        Notes
        -----
        `copy.deepcopy` is used, `normalize_to_meters` is then called on the copy.
        """

        new_target = deepcopy(self)
        new_target.normalize_to_meters()
        return new_target


class FrameTarget(Target):
    """Represents a fully constrained pose target for the robot's end-effector using a :class:`compas.geometry.Frame`.

    When using a FrameTarget, the end-effector has no translational or rotational freedom.
    In another words, the pose of the end-effector is fully defined (constrained).

    Given a FrameTarget, the planner aims to find a path moving
    the robot's Tool0 Coordinate Frame (T0CF) to the specified `FrameTarget.target_frame`.

    For robots with multiple end effector attachment points (such as the RFL Robot), the attachment point depends on
    the planning group selected in the planning request, see :meth:`compas_fab.robots.Robot.plan_motion`.

    Attributes
    ----------
    target_frame : :class:`compas.geometry.Frame`
        The target frame.
    target_mode : :class:`TargetMode` or str
        The target mode specifies which link or frame is referenced when specifying a target.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.
    tolerance_position : float, optional
        The tolerance for the position.
        Unit is meters.
        If not specified, the default value from the planner is used.
    tolerance_orientation : float, optional
        The tolerance for the orientation.
        Unit is radians.
        If not specified, the default value from the planner is used.
    name : str, optional
        The human-readable name of the target.
        Defaults to 'Frame Target'.

    """

    def __init__(
        self,
        target_frame,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Frame Target",
    ):
        # type: (Frame, TargetMode | str, Optional[float], Optional[float], Optional[float], Optional[str]) -> None
        super(FrameTarget, self).__init__(target_mode=target_mode, target_scale=target_scale, name=name)
        self.target_frame = target_frame
        self.tolerance_position = tolerance_position
        self.tolerance_orientation = tolerance_orientation

    @property
    def __data__(self):
        return {
            "target_frame": self.target_frame,
            "target_mode": self.target_mode,
            "target_scale": self.target_scale,
            "tolerance_position": self.tolerance_position,
            "tolerance_orientation": self.tolerance_orientation,
            "name": self.name,
        }

    @classmethod
    def from_transformation(
        cls,
        transformation,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Frame Target",
    ):
        # type: (Transformation, TargetMode | str, Optional[float], Optional[float], Optional[float], Optional[str]) -> FrameTarget
        """Creates a FrameTarget from a transformation matrix.

        Parameters
        ----------
        transformation : :class
            The transformation matrix.
        target_mode : :class:`TargetMode` or str
            The target mode specifies which link or frame is referenced when specifying a target.
            See :class:`TargetMode` for more details.
        target_scale : float, optional
            The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
            Defaults to 1.0.
        tolerance_position : float, optional
            The tolerance for the position.
            if not specified, the default value from the planner is used.
        tolerance_orientation : float, optional
            The tolerance for the orientation.
            if not specified, the default value from the planner is used.
        name : str, optional
            The human-readable name of the target.
            Defaults to 'Frame Target'.

        Returns
        -------
        :class:`FrameTarget`
            The frame target.
        """
        frame = Frame.from_transformation(transformation)
        return cls(frame, target_mode, target_scale, tolerance_position, tolerance_orientation, name)

    def normalize_to_meters(self):
        # type: () -> None
        """Convert the target into meter scale if `target_scale` is not 1.0.

        Because all robots and planners in compas_fab use meters as the default unit of measure,
        user targets that are created in other units (e.g. millimeters) will have a `target_scale`
        factor such as 0.001 for millimeters.

        This function convert the `target_frame` and `tolerance_position` into meters scale.
        `target_scale` is set to 1.0 after the conversion.

        """
        # Skipping the conversion if the target_scale is already 1.0
        if self.target_scale == 1.0:
            return

        self.target_frame.scale(self.target_scale)
        self.tolerance_position = self.tolerance_position * self.target_scale if self.tolerance_position else None
        # NOTE: tolerance_orientation is not scaled
        self.target_scale = 1.0

    def __eq__(self, other):
        # type: (FrameTarget) -> bool
        """Check if two FrameTarget objects are equal.

        This function relies on the `is_close` function from the `compas.tolerance` module.
        Hence, the numerical values of the geometry are compared with the globally defined tolerance.
        """

        # NOTE: Some attributes are optional, so we need to check if they are equally None
        return (
            TOL.is_allclose(other.target_frame, self.target_frame)
            and self.target_mode == other.target_mode
            and (self.target_scale == other.target_scale or TOL.is_close(other.target_scale, self.target_scale))
            and (
                self.tolerance_position == other.tolerance_position
                or TOL.is_close(other.tolerance_position, self.tolerance_position)
            )
            and (
                self.tolerance_orientation == other.tolerance_orientation
                or TOL.is_close(other.tolerance_orientation, self.tolerance_orientation)
            )
            and other.name == self.name
        )


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
    Note that PointAxisTarget only represents a single target,
    for a sequence of targets, consider using :class:`PointAxisWaypoints`.

    The user must define (1) the target point of which the tool tip will reach
    and (2) the target axis where the tool tip coordinate frame (TCF)'s Z axis
    can rotate around. The target point and axis are defined relative to the robot's
    world coordinate frame (WCF).

    For robots with multiple end effector attachment points, the FCF depends on
    the planning group setting in the planning request, as defined in an SRDF file or
    :class:`compas_fab.robots.RobotSemantics`.

    Attributes
    ----------
    target_point : :class:`compas.geometry.Point`
        The target point defined relative to the world coordinate frame (WCF).
    target_z_axis : :class:`compas.geometry.Vector`
        The target axis is defined by the target_point and pointing towards this vector.
        A unitized vector is recommended.
        The tool tip coordinate frame (TCF)'s Z axis can rotate around this axis.
    target_mode : :class:`TargetMode` or str
        The target mode specifies which link or frame is referenced when specifying a target.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.
    tolerance_position : float, optional
        The tolerance for the position of the target point.
        Unit is meters.
        If not specified, the default value from the planner is used.
    tolerance_orientation : float, optional
        The tolerance for matching the target axis orientation.
        Unit is in radians.
        If not specified, the default value from the planner is used.
    name : str, optional
        The human-readable name of the target.
        Defaults to 'Point-Axis Target'.
    """

    def __init__(
        self,
        target_point,
        target_z_axis,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Point-Axis Target",
    ):
        # type: (Point, Vector, TargetMode | str, Optional[float], Optional[float], Optional[float], Optional[str]) -> None
        super(PointAxisTarget, self).__init__(target_mode=target_mode, target_scale=target_scale, name=name)
        # Note: The following input are converted to class because it can simplify functions that use this class
        self.target_point = Point(*target_point)
        self.target_z_axis = Vector(*target_z_axis).unitized()
        self.tolerance_position = tolerance_position
        self.tolerance_orientation = tolerance_orientation

    @property
    def __data__(self):
        return {
            "target_point": self.target_point,
            "target_mode": self.target_mode,
            "target_scale": self.target_scale,
            "target_z_axis": self.target_z_axis,
            "tolerance_position": self.tolerance_position,
            "tolerance_orientation": self.tolerance_orientation,
            "name": self.name,
        }

    def normalize_to_meters(self):
        # type: () -> None
        """Convert the target into meter scale if `target_scale` is not 1.0.

        Because all robots and planners in compas_fab use meters as the default unit of measure,
        user targets that are created in other units (e.g. millimeters) will have a `target_scale`
        factor such as 0.001 for millimeters.

        This function convert the `target_point`, `target_z_axis` and `tolerance_position` into meters scale.
        `target_scale` is set to 1.0 after the conversion.

        """
        # Skipping the conversion if the target_scale is already 1.0
        if self.target_scale == 1.0:
            return

        self.target_point.scale(self.target_scale)
        self.tolerance_position = self.tolerance_position * self.target_scale if self.tolerance_position else None
        # NOTE: tolerance_orientation is not scaled
        self.target_z_axis.unitize()
        # NOTE: target_z_axis is unitized and is not scaled
        self.target_scale = 1.0

    def __eq__(self, other):
        # type: (PointAxisTarget) -> bool
        """Check if two PointAxisTarget objects are equal.

        This function relies on the `is_close` function from the `compas.tolerance` module.
        Hence, the numerical values of the geometry are compared with the globally defined tolerance.
        """

        # NOTE: Some attributes are optional, so we need to check if they are equally None
        return (
            TOL.is_allclose(other.target_point, self.target_point)
            and TOL.is_allclose(other.target_z_axis, self.target_z_axis)
            and self.target_mode == other.target_mode
            and (other.target_scale == self.target_scale or TOL.is_close(other.target_scale, self.target_scale))
            and (
                other.tolerance_position == self.tolerance_position
                or TOL.is_close(other.tolerance_position, self.tolerance_position)
            )
            and (
                other.tolerance_orientation == self.tolerance_orientation
                or TOL.is_close(other.tolerance_orientation, self.tolerance_orientation)
            )
            and other.name == self.name
        )


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
        Units must be in meters for prismatic joints and radians for revolute and continuous joints.
        If not specified, the default value from the planner is used.
    tolerance_below : :obj:`list` of :obj:`float`, optional
        Acceptable deviation below the targeted configurations. One for each joint.
        Always use positive values.
        Units must be in meters for prismatic joints and radians for revolute and continuous joints.
        If not specified, the default value from the planner is used.
    name : str, optional
        The human-readable name of the target.
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

    @property
    def __data__(self):
        return {
            "target_configuration": self.target_configuration,
            "tolerance_above": self.tolerance_above,
            "tolerance_below": self.tolerance_below,
            "name": self.name,
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

    # The following function is retired because we no longer support scaling the tolerance of a ConfigurationTarget
    # Users who use ConfigurationTarget should set the tolerance values using the native units of the robot model.

    # def scaled(self, factor):
    #     # type: (float) -> ConfigurationTarget
    #     """Returns copy of the target where the target configuration and tolerances are scaled.

    #     This function should only be needed if the ConfigurationTarget was created
    #     with a distance unit other than meters.

    #     Only the values for prismatic and planar joints are scaled. The values for revolute
    #     and continuous joints are not scaled, as they must be in radians.

    #     Parameters
    #     ----------
    #     factor : float
    #         The scaling factor.

    #     Returns
    #     -------
    #     :class:`ConfigurationTarget`
    #         The scaled configuration target.
    #     """
    #     target_configuration = self.target_configuration.scaled(factor)

    #     def scale_tolerance(tolerance, joint_types):
    #         # type: (list[float], list[Joint]) -> list[float]
    #         """Only scales the tolerances for prismatic and planar joints."""
    #         scaled_tolerance = []
    #         for t, joint_type in zip(tolerance, joint_types):
    #             if joint_type in (Joint.PLANAR, Joint.PRISMATIC):
    #                 t *= factor
    #             scaled_tolerance.append(t)
    #         return scaled_tolerance

    #     # We scale only the tolerances for prismatic and planar joints,
    #     # similar to the Configuration.scale() method
    #     tolerance_above = (
    #         scale_tolerance(self.tolerance_above, target_configuration.joint_types) if self.tolerance_above else None
    #     )
    #     tolerance_below = (
    #         scale_tolerance(self.tolerance_below, target_configuration.joint_types) if self.tolerance_below else None
    #     )

    #     return ConfigurationTarget(target_configuration, tolerance_above, tolerance_below, self.name)

    def normalize_to_meters():
        """ConfigurationTarget does not contain any geometry with configurable units to normalize."""
        pass

    def __eq__(self, other):
        # type: (ConfigurationTarget) -> bool
        """Check if two ConfigurationTarget objects are equal.

        This function relies on the `is_close` function from the `compas.tolerance` module.
        Hence, the numerical values of the geometry are compared with the globally defined tolerance.

        Except where the target_configurations are compared with `Configuration.is_close` method.
        """

        # NOTE: Some attributes are optional, so we need to check if they are equally None
        return (
            self.target_configuration.close_to(other.target_configuration)
            and (
                other.tolerance_above == self.tolerance_above
                or TOL.is_allclose(other.tolerance_above, self.tolerance_above)
            )
            and (
                other.tolerance_below == self.tolerance_below
                or TOL.is_allclose(other.tolerance_below, self.tolerance_below)
            )
            and other.name == self.name
        )


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
        The human-readable name of the target.
        Defaults to 'Constraint Set Target'.
    """

    def __init__(self, constraint_set, name="Constraint Set Target"):
        # type: (list[Constraint], Optional[str]) -> None
        super(ConstraintSetTarget, self).__init__(name=name)
        self.constraint_set = constraint_set

    @property
    def __data__(self):
        return {
            "constraint_set": self.constraint_set,
            "name": self.name,
        }

    def normalize_to_meters():
        """ConstraintSetTarget does not contain any geometry with configurable units to normalize."""
        pass


class Waypoints(Target):
    """Represents a sequence of kinematic target for motion planning.

    Waypoints represent a sequence of targets the robot should pass through in the order they are defined.
    This is in contrast to :class:`Target` which represent only a single target.
    The initial (starting) point should not be included in the waypoints list.
    It is valid for a Waypoints object to have one target.

    Waypoints are useful for tasks like painting, welding, or 3D printing, where the programmer
    wants to define the waypoints the robot should pass through.

    Waypoints are intended to be used for motion planning with a planning backend by using :meth:`compas_fab.robot.plan_cartesian_motion`.
    Note that different backends support different types of waypoints.
    The method of interpolation between the waypoints is controlled by the motion planner backend.

    Attributes
    ----------
    target_mode : :class:`TargetMode` or str, optional
        The target mode specifies which link or frame is referenced when specifying a target.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.
    name : str , optional, default = 'target'
        A human-readable name for identifying the target.

    See Also
    --------
    :class:`PointAxisWaypoints`
    :class:`FrameWaypoints`
    """

    def __init__(self, target_mode, target_scale=1.0, name="Generic Waypoints"):
        # type: (Optional[TargetMode | str], Optional[float], Optional[str]) -> None
        super(Waypoints, self).__init__(target_mode=target_mode, target_scale=target_scale, name=name)


class FrameWaypoints(Waypoints):
    """Represents a sequence of fully constrained pose target for the robot's end-effector using a :class:`compas.geometry.Frame`.

    When using a FrameWaypoints, the end-effector has no translational or rotational freedom.
    In another words, the pose of the end-effector is fully defined (constrained).

    The behavior of FrameWaypoints is similar to :class:`FrameTarget`, but it represents a sequence of targets.

    Attributes
    ----------
    target_frames : :obj:`list` of :class:`compas.geometry.Frame`
        The target frames.
    target_mode : :class:`TargetMode` or str
        The target mode specifies which link or frame is referenced when specifying a target.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.
    tolerance_position : float, optional
        The tolerance for the position.
        Unit is meters.
        If not specified, the default value from the planner is used.
    tolerance_orientation : float, optional
        The tolerance for the orientation.
        Unit is radians.
        If not specified, the default value from the planner is used.
    name : str, optional
        The human-readable name of the target.
        Defaults to 'Frame Waypoints'.

    """

    def __init__(
        self,
        target_frames,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Frame Waypoints",
    ):
        # type: (list[Frame], TargetMode | str, Optional[float], Optional[float], Optional[float], Optional[str]) -> None
        super(FrameWaypoints, self).__init__(target_mode=target_mode, target_scale=target_scale, name=name)
        self.target_frames = target_frames
        self.tolerance_position = tolerance_position
        self.tolerance_orientation = tolerance_orientation

    @property
    def __data__(self):
        return {
            "target_frames": self.target_frames,
            "target_mode": self.target_mode,
            "target_scale": self.target_scale,
            "tolerance_position": self.tolerance_position,
            "tolerance_orientation": self.tolerance_orientation,
            "name": self.name,
        }

    @classmethod
    def from_transformations(
        cls,
        transformations,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Frame Waypoints",
    ):
        # type: (list[Transformation], TargetMode | str, Optional[float],  Optional[float], Optional[float], Optional[str]) -> FrameWaypoints
        """Creates a FrameWaypoints from a list of transformation matrices.

        Parameters
        ----------
        transformations : :obj:`list` of :class: `compas.geometry.Transformation`
            The list of transformation matrices.
        target_mode : :class:`TargetMode` or str
            The target mode specifies which link or frame is referenced when specifying a target.
            See :class:`TargetMode` for more details.
        target_scale : float, optional
            The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
            Defaults to 1.0.
        tolerance_position : float, optional
            The tolerance for the position.
            if not specified, the default value from the planner is used.
        tolerance_orientation : float, optional
            The tolerance for the orientation.
            if not specified, the default value from the planner is used.
        name : str, optional
            The human-readable name of the target.
            Defaults to 'Frame Target'.

        Returns
        -------
        :class:`FrameWaypoints`
            The frame waypoints.
        """
        frames = [Frame.from_transformation(transformation) for transformation in transformations]
        return cls(frames, target_mode, target_scale, tolerance_position, tolerance_orientation, name)

    def normalize_to_meters(self):
        # type: () -> None
        """Convert the target into meter scale if `target_scale` is not 1.0.

        Because all robots and planners in compas_fab use meters as the default unit of measure,
        user targets that are created in other units (e.g. millimeters) will have a `target_scale`
        factor such as 0.001 for millimeters.

        This function convert the `target_frame` and `tolerance_position` into meters scale.
        `target_scale` is set to 1.0 after the conversion.

        """
        # Skipping the conversion if the target_scale is already 1.0
        if self.target_scale == 1.0:
            return

        for frame in self.target_frames:
            frame.scale(self.target_scale)
        self.tolerance_position = self.tolerance_position * self.target_scale if self.tolerance_position else None
        # NOTE: tolerance_orientation is not scaled
        self.target_scale = 1.0

    def __eq__(self, other):
        # type: (FrameWaypoints) -> bool
        """Check if two FrameWaypoints objects are equal.

        This function relies on the `is_close` function from the `compas.tolerance` module.
        Hence, the numerical values of the geometry are compared with the globally defined tolerance.
        """
        return (
            len(self.target_frames) == len(other.target_frames)
            and all(
                TOL.is_allclose(other_frame, self_frame)
                for other_frame, self_frame in zip(other.target_frames, self.target_frames)
            )
            and self.target_mode == other.target_mode
            and TOL.is_close(other.target_scale, self.target_scale)
            and TOL.is_close(other.tolerance_position, self.tolerance_position)
            and TOL.is_close(other.tolerance_orientation, self.tolerance_orientation)
            and other.name == self.name
        )


class PointAxisWaypoints(Waypoints):
    """
    Represents a sequence of point and axis targets for the robot's end-effector motion planning.

    PointAxisTarget is suitable for tasks like drawing, milling, and 3D printing,
    where aligning the end-effector with a target axis is crucial,
    but the orientation around the axis is not important.

    The behavior of PointAxisWaypoints is similar to :class:`PointAxisTarget`, but it represents a sequence of targets.
    See :class:`PointAxisTarget` for more details.

    Attributes
    ----------
    target_points_and_axes : :obj:`list` of :obj:`tuple` of (:class:`compas.geometry.Point`, :class:`compas.geometry.Vector`)
        The target points and axes.
        Both values are defined relative to the world coordinate frame (WCF).
        Unitized vectors are recommended for the target axes.
    target_mode : :class:`TargetMode` or str
        The target mode specifies which link or frame is referenced when specifying a target.
        See :class:`TargetMode` for more details.
    target_scale : float, optional
        The scaling factor for the target frame. Use 1.0 for meters, 0.001 for millimeters, etc.
        Defaults to 1.0.
    tolerance_position : float, optional
        The tolerance for the position of the target point.
        Unit is meters.
        If not specified, the default value from the planner is used.
    tolerance_orientation : float, optional
        The tolerance for matching the target axis orientation.
        Unit is in radians.
        If not specified, the default value from the planner is used.
    name : str, optional
        The human-readable name of the target.
        Defaults to 'Point-Axis Waypoints'.

    """

    def __init__(
        self,
        target_points_and_axes,
        target_mode,
        target_scale=1.0,
        tolerance_position=None,
        tolerance_orientation=None,
        name="Point-Axis Waypoints",
    ):
        # type: (list[Tuple[Point, Vector]], TargetMode | str, Optional[float], Optional[float], Optional[float], Optional[str]) -> None
        super(PointAxisWaypoints, self).__init__(target_mode=target_mode, target_scale=target_scale, name=name)
        self.target_points_and_axes = target_points_and_axes
        self.tolerance_position = tolerance_position
        self.tolerance_orientation = tolerance_orientation

    @property
    def __data__(self):
        return {
            "target_points_and_axes": self.target_points_and_axes,
            "target_mode": self.target_mode,
            "target_scale": self.target_scale,
            "tolerance_position": self.tolerance_position,
            "tolerance_orientation": self.tolerance_orientation,
            "name": self.name,
        }

    def normalize_to_meters(self):
        # type: () -> None
        """Convert the target into meter scale if `target_scale` is not 1.0.

        Because all robots and planners in compas_fab use meters as the default unit of measure,
        user targets that are created in other units (e.g. millimeters) will have a `target_scale`
        factor such as 0.001 for millimeters.

        This function convert the Points and Vectors in `target_points_and_axes` into meters scale.
        `target_scale` is set to 1.0 after the conversion.

        """
        # Skipping the conversion if the target_scale is already 1.0
        if self.target_scale == 1.0:
            return

        for point, axis in self.target_points_and_axes:
            point.scale(self.target_scale)
            axis.unitize()
            # NOTE: target_z_axis is unitized and is not scaled

        self.tolerance_position = self.tolerance_position * self.target_scale if self.tolerance_position else None
        # NOTE: tolerance_orientation is not scaled
        self.target_scale = 1.0

    def __eq__(self, other):
        # type: (PointAxisWaypoints) -> bool
        """Check if two FrameWaypoints objects are equal.

        This function relies on the `is_close` function from the `compas.tolerance` module.
        Hence, the numerical values of the geometry are compared with the globally defined tolerance.
        """
        return (
            len(self.target_points_and_axes) == len(other.target_points_and_axes)
            and all(
                TOL.is_allclose(other_point, self_point) and TOL.is_allclose(other_axis, self_axis)
                for (other_point, other_axis), (self_point, self_axis) in zip(
                    other.target_points_and_axes, self.target_points_and_axes
                )
            )
            and self.target_mode == other.target_mode
            and TOL.is_close(other.target_scale, self.target_scale)
            and TOL.is_close(other.tolerance_position, self.tolerance_position)
            and TOL.is_close(other.tolerance_orientation, self.tolerance_orientation)
            and other.name == self.name
        )


class TargetMode:
    """Represents different matching mode for `Targets` and `Waypoints`.

    Target modes represent which link or frame is referenced when specifying a target or waypoint.
    For example, if a target's  `target_mode` is `TargetMode.TCF` (Tool Coordinate Frame),
    it means that the planner will try to move the robot, such that it's attached TCF
    matches with the `.frame` specified in the Target or Waypoint.

    Attributes
    ----------
    ROBOT : str
        Refers to the PCF (Planner Coordinate Frame).
        The frame of the tip link of a planning group.
    TOOL : str
        Refers to the TCF (Tool Coordinate Frame) of the tool attached to the robot.
        Typically this frame is at the tool tip of the tool.
        A tool must be attached to the robot to use this.
    WORKPIECE : str
        Refers to the frame of the workpiece (:class:`~compas_fab.robots.RigidBody`) attached to the robot.
        There must be one and only one workpiece attached to the robot when using this mode.

    Notes
    -----
    The term `workpiece` refers to the RigidBody attached to a tool.

    When using the `WORKPIECE` mode, the user must ensure that only one workpiece is attached to the robot.
    Otherwise, it is not possible for compas_fab to determine which workpiece is being referred to.
    If the user has multiple workpieces, they should use the `TOOL` mode instead.
    """

    ROBOT = "ROBOT"
    TOOL = "TOOL"
    WORKPIECE = "WORKPIECE"
