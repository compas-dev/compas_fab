from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas_robots import Configuration
from compas_fab.robots import Constraint


from compas.datastructures import Mesh

try:
    from typing import List
except ImportError:
    pass


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
        super(Target, self).__init__()
        self.name = name


class FrameTarget(Target):
    """Represents a fully constrained pose target for the robot's end-effector using a :class:`compas.geometry.Frame`.

    The end-effector has no translational or rotational freedom.

    Given a FrameTarget, the planner aims to find a path moving
    the robot's flange frame (or tool frame) to the specified `FrameTarget.target_frame`.
    The default target frame corresponds to the Flange Coordinate Frame (RfCF)
    of the robot. When using a tool, the target frame can be specified
    relative to the tool tip coordinate frame (TTCF).
    The planner will then move the tool tip towards the target frame.

    For robots with multiple end effector attachment points, the FCF depends on
    the planning group setting in the planning request, as defined in an SRDF file or
    :class:`compas_fab.robots.RobotSemantics`.
    Attributes
    ----------
    target_frame : :class:`compas.geometry.Frame`
        The target frame.
    F_TtCF : :class:`compas.geometry.Frame`, optional
        The tool tip coordinate frame relative to the flange of the robot.
        If not specified, the target frame is relative to the robot's flange.

    """

    def __init__(self, target_frame, F_TtCF=None, name="Frame Target"):
        super(FrameTarget, self).__init__(name=name)
        self.F_TtCF = F_TtCF
        self.target_frame = target_frame


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
    base coordinate frame (RCF).

    In addition, it's necessary to define the tool tip coordinate frame (TtCF)
    relative to the robot's flange frame (FCF). This is labeled as F_TtCF.
    If F_TtCF is unspecified, the target point and axis will be matched with the robot's flange frame.

    For robots with multiple end effector attachment points, the FCF depends on
    the planning group setting in the planning request, as defined in an SRDF file or
    :class:`compas_fab.robots.RobotSemantics`.

    Attributes
    ----------
    target_point : :class:`compas.geometry.Point`
        The target point defined in the robot's base coordinate frame (RCF).
    target_z_vector : :class:`compas.geometry.Vector`
        The target axis defined in the robot's base coordinate frame (RCF).
    F_TtCF : :class:`compas.geometry.Frame`, optional
        The tool tip coordinate frame relative to the flange coordinate frame of the robot.
        If not specified, the target point is relative to the robot's flange.

    """

    def __init__(self, target_point, target_z_vector, F_TtCF=None, name="Point-Axis Target"):
        super(PointAxisTarget, self).__init__(name=name)
        self.F_TtCF = F_TtCF  # type: Frame
        self.target_point = target_point  # type: Point
        self.target_z_vector = target_z_vector  # type: Vector


class ConfigurationTarget(Target):
    """Represents a configuration target for the robot's end-effector motion planning.

    The configuration target is a joint configuration of the robot's joints.
    Given a ConfigurationTarget, the planner aims to find a path moving
    the robot's joint positions to match with `ConfigurationTarget.target_configuration`.

    ConfigurationTarget is suitable for targets whose joint configuration is known,
    such as a home position, or a repetitive position that has been calibrated
    in the actual robot cell, such as a tool changing position.

    The number of joints in the target configuration should match the number of joints
    in the robot's planning group. Otherwise the behaviour of the bankend planner may
    be undefined. See tutorial :ref:`configuration-target` for more details.

    Attributes
    ----------
    target_configuration : :class:`compas_robots.Configuration`
        The target configuration. joint_names and joint_values must be specified.
    """

    def __init__(self, target_configuration, name="Configuration Target"):
        super(ConfigurationTarget, self).__init__(name=name)
        self.target_configuration = target_configuration  # type: Configuration


class ConstraintSetTarget(Target):
    """Represents a list of Constraint as the target for motion planning.

    Given a ConstraintSetTarget, the planner aims to find a path moving
    the robot's end-effector to satisfy ALL the constraints in the constraint set.
    This target cannot be translated into a single pose or configuration target
    except in trivial cases. Therefore the ending pose or configuration of the
    planned path can only be determined after performing the motion planning.

    ConstraintSetTarget cannot be used as a target for Cartesian motion planning.

    ConstraintSetTarget is suitable for advanced users who want to specify
    custom constraints for the robot motion planning.
    Different planner backends may support differnt types of Constraints.
    See tutorial :ref:`constraint-set-target` for more details.

    Attributes
    ----------
    constraint_set : :class:`compas_fab.robots.Constraint`
        A list of constraints to be satisfied.
    """

    def __init__(self, constraint_set, name="Constraint Set Target"):
        super(ConstraintSetTarget, self).__init__(name=name)
        self.constraint_set = constraint_set  # type: List[Constraint]
