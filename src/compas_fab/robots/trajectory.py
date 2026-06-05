from typing import TYPE_CHECKING
from typing import Any
from typing import Optional
from typing import cast

from compas_fab.robots import TargetMode

if TYPE_CHECKING:
    from compas_fab.robots import RobotCell
    from compas_fab.robots import RobotCellState

from compas.data import Data
from compas.geometry import Polyline
from compas.tolerance import TOL
from compas_robots import Configuration
from compas_robots.configuration import FixedLengthList

from compas_fab.robots.time_ import Duration

__all__ = [
    "JointTrajectory",
    "JointTrajectoryPoint",
    "Trajectory",
]


class JointTrajectoryPoint(Configuration):
    """Defines a point within a trajectory.

    A trajectory point is a sub-class of [Configuration][compas_robots.Configuration]
    extended with acceleration, effort and time-from-start information.

    Trajectory points are defined either as *joint_values + velocities and
    accelerations*, or as *joint_values + efforts*.

    Parameters
    ----------
    joint_values
        Joint values expressed in radians or meters, depending on the joint
        type.
    joint_types
        Joint types, e.g. a list of `Joint.REVOLUTE` for revolute joints.
    velocities
        Velocity of each joint.
    accelerations
        Acceleration of each joint.
    effort
        Effort of each joint.
    time_from_start
        Duration of the trajectory point counted from the start.
    joint_names
        Names of the joints, in the same order as `joint_values`.

    Attributes
    ----------
    positions
        Alias of `joint_values`.
    data
        The data representing the trajectory point.
    """

    def __init__(
        self,
        joint_values: Optional[list[float]] = None,
        joint_types: Optional[list[int]] = None,
        velocities: Optional[list[float]] = None,
        accelerations: Optional[list[float]] = None,
        effort: Optional[list[float]] = None,
        time_from_start: Optional[Duration] = None,
        joint_names: Optional[list[str]] = None,
    ):
        super(JointTrajectoryPoint, self).__init__(joint_values, joint_types, joint_names)
        self.velocities = velocities or len(self.joint_values) * [0.0]
        self.accelerations = accelerations or len(self.joint_values) * [0.0]
        self.effort = effort or len(self.joint_values) * [0.0]
        self.time_from_start = time_from_start or Duration(0, 0)

    def __str__(self):
        """Return a human-readable string representation of the instance."""
        return "JointTrajectoryPoint(({}), {}, ({}), ({}), ({}), {})".format(
            ", ".join(TOL.format_number(i) for i in self.joint_values),
            tuple(self.joint_types),
            ", ".join(TOL.format_number(i) for i in self.velocities),
            ", ".join(TOL.format_number(i) for i in self.accelerations),
            ", ".join(TOL.format_number(i) for i in self.effort),
            self.time_from_start,
        )

    @property
    def positions(self) -> list[float]:
        """Alias of `joint_values`."""
        return self.joint_values

    @property
    def velocities(self) -> list[float]:
        """Velocity of each joint."""
        return self._velocities

    @velocities.setter
    def velocities(self, velocities: list[float]):
        if len(self.joint_values) != len(velocities):
            raise ValueError("Must have {} velocities, but {} given.".format(len(self.joint_values), len(velocities)))

        self._velocities = FixedLengthList(velocities)

    @property
    def accelerations(self) -> list[float]:
        """Acceleration of each joint."""
        return self._accelerations

    @accelerations.setter
    def accelerations(self, accelerations: list[float]):
        if len(self.joint_values) != len(accelerations):
            raise ValueError("Must have {} accelerations, but {} given.".format(len(self.joint_values), len(accelerations)))

        self._accelerations = FixedLengthList(accelerations)

    @property
    def effort(self) -> list[float]:
        """Effort of each joint."""
        return self._effort

    @effort.setter
    def effort(self, effort: list[float]):
        if len(self.joint_values) != len(effort):
            raise ValueError("Must have {} efforts, but {} given.".format(len(self.joint_values), len(effort)))

        self._effort = FixedLengthList(effort)

    @property
    def __data__(self):
        """The serialised trajectory point.

        Assigning a dict to this property replaces the current data; the
        getter and setter must always be used together.
        """
        data_obj = super(JointTrajectoryPoint, self).__data__
        data_obj["velocities"] = self.velocities
        data_obj["accelerations"] = self.accelerations
        data_obj["effort"] = self.effort
        data_obj["time_from_start"] = self.time_from_start.__data__

        return data_obj

    @classmethod
    def __from_data__(cls, data: dict[str, Any]) -> "JointTrajectoryPoint":
        joint_values = FixedLengthList(data.get("joint_values") or data.get("values") or [])
        joint_types = FixedLengthList(data.get("joint_types") or data.get("types") or [])
        joint_names = FixedLengthList(data.get("joint_names") or [])
        velocities = FixedLengthList(data.get("velocities") or [])
        accelerations = FixedLengthList(data.get("accelerations") or [])
        effort = FixedLengthList(data.get("effort") or [])
        time_from_start = Duration.__from_data__(data.get("time_from_start") or {})

        tool_trajectory_point = cls(
            joint_values=joint_values,
            joint_types=joint_types,
            velocities=velocities,
            accelerations=accelerations,
            effort=effort,
            time_from_start=time_from_start,
            joint_names=joint_names,
        )
        return tool_trajectory_point

    @property
    def velocity_dict(self) -> dict[str, float]:
        """A dictionary of joint velocities by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.velocities))

    @property
    def acceleration_dict(self) -> dict[str, float]:
        """A dictionary of joint accelerations by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.accelerations))

    @property
    def effort_dict(self) -> dict[str, float]:
        """A dictionary of joint efforts by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.effort))

    def merged(self, other) -> "JointTrajectoryPoint":
        """Return a new point with `self` merged with `other`.

        `other` takes precedence over `self` for any joint present in both.
        Joint names in the returned point may be reordered.

        Parameters
        ----------
        other
            The point to be merged.

        Raises
        ------
        ValueError
            If either point does not specify joint names for all joint values.
        """
        _joint_dict = self.joint_dict
        _joint_dict.update(other.joint_dict)

        _type_dict = self.type_dict
        _type_dict.update(other.type_dict)

        _velocity_dict = self.velocity_dict
        _velocity_dict.update(other.velocity_dict)

        _acceleration_dict = self.acceleration_dict
        _acceleration_dict.update(other.acceleration_dict)

        _effort_dict = self.effort_dict
        _effort_dict.update(other.effort_dict)

        joint_names = list(_joint_dict.keys())
        joint_values = [_joint_dict[name] for name in joint_names]
        joint_types = [_type_dict[name] for name in joint_names]
        velocities = [_velocity_dict[name] for name in joint_names]
        accelerations = [_acceleration_dict[name] for name in joint_names]
        effort = [_effort_dict[name] for name in joint_names]

        return JointTrajectoryPoint(joint_values, joint_types, velocities, accelerations, effort, joint_names=joint_names)


class Trajectory(Data):
    """Base trajectory class.

    Attributes
    ----------
    planning_time
        Amount of time it took to complete the motion plan, in seconds.
        Defaults to `-1` when the backend did not report it.
    attributes
        Custom attributes of the trajectory.
    """

    def __init__(self, attributes: Optional[dict[str, Any]] = None):
        super(Trajectory, self).__init__()
        self.attributes = attributes or {}
        self.planning_time = -1

    @property
    def __data__(self):
        data_obj = {}
        data_obj["planning_time"] = self.planning_time
        data_obj["attributes"] = self.attributes
        return data_obj

    @classmethod
    def __from_data__(cls, data: dict[str, Any]) -> "Trajectory":
        trajectory = cls(attributes=data["attributes"])
        trajectory.planning_time = data["planning_time"]
        return trajectory


class JointTrajectory(Trajectory):
    """Describes a joint trajectory as a list of trajectory points.

    Parameters
    ----------
    trajectory_points
        List of points composing the trajectory.
    joint_names
        List of joint names of the trajectory.
    start_state
        The full robot cell state the planner was given (tools, rigid bodies,
        attached collision objects, robot configuration). When present,
        callers no longer have to thread the cell state separately through
        visualisation / replay code.
    fraction
        Percentage of the requested trajectory that was calculated, e.g. `1`
        means the full trajectory was found.
    attributes
        Custom attributes of the trajectory.

    Attributes
    ----------
    points
        List of points composing the trajectory.
    start_configuration
        Start configuration for the trajectory. Not a constructor parameter:
        assign via the property after construction to override. The getter
        returns the explicit override when set, otherwise
        `start_state.robot_configuration`, otherwise `None`.
    data
        The serialised trajectory.
    """

    def __init__(
        self,
        trajectory_points: Optional[list[JointTrajectoryPoint]] = None,
        joint_names: Optional[list[str]] = None,
        start_state: Optional["RobotCellState"] = None,
        fraction: Optional[float] = None,
        attributes: Optional[dict[str, Any]] = None,
    ):
        super(JointTrajectory, self).__init__(attributes=attributes)
        self.points = trajectory_points or []
        self.joint_names = joint_names or []
        self._start_configuration: Optional[Configuration] = None
        self.start_state = start_state
        self.fraction = fraction

    @property
    def start_configuration(self) -> Optional[Configuration]:
        if self._start_configuration is not None:
            return self._start_configuration
        if self.start_state is not None:
            return self.start_state.robot_configuration
        return None

    @start_configuration.setter
    def start_configuration(self, value: Optional[Configuration]) -> None:
        self._start_configuration = value

    @property
    def __data__(self):
        """The serialised trajectory."""
        data_obj = {}
        data_obj["points"] = [p.__data__ for p in self.points]
        data_obj["joint_names"] = self.joint_names or []
        data_obj["start_configuration"] = self._start_configuration.__data__ if self._start_configuration else None
        data_obj["start_state"] = self.start_state.__data__ if self.start_state else None
        data_obj["fraction"] = self.fraction
        data_obj["planning_time"] = self.planning_time
        data_obj["attributes"] = self.attributes

        return data_obj

    @classmethod
    def __from_data__(cls, data: dict[str, Any]) -> "JointTrajectory":
        from compas_fab.robots import RobotCellState

        points = list(map(JointTrajectoryPoint.__from_data__, data.get("points") or []))
        joint_names = data.get("joint_names", [])
        start_configuration_data = data.get("start_configuration", None)
        start_configuration = cast(
            "Optional[Configuration]",
            Configuration.__from_data__(start_configuration_data) if start_configuration_data else None,
        )
        start_state_data = data.get("start_state", None)
        start_state = cast("Optional[RobotCellState]", RobotCellState.__from_data__(start_state_data) if start_state_data else None)
        fraction = data.get("fraction")

        trajectory = cls(
            trajectory_points=points,
            joint_names=joint_names,
            start_state=start_state,
            fraction=fraction,
            attributes=data["attributes"],
        )
        if start_configuration is not None:
            trajectory.start_configuration = start_configuration
        trajectory.planning_time = data["planning_time"]

        return trajectory

    @property
    def time_from_start(self) -> float:
        """Time from start of the last point — effectively the trajectory's total duration in seconds. Zero when empty."""
        if not self.points:
            return 0.0

        return self.points[-1].time_from_start.seconds

    def to_frames_and_polyline(self, robot_cell: "RobotCell", start_state: Optional["RobotCellState"], group=None):
        """Compute frames and a connecting polyline from a trajectory.

        For each trajectory point, the helper substitutes the point's joint
        values into a single working copy of `start_state` and runs **local**
        forward kinematics via `RobotCell.forward_kinematics_target_frame`
        (`target_mode=ROBOT`), i.e. the URDF kinematic chain in-process, with
        no backend round trip. A polyline through their origins is computed
        as well.

        `start_state` may be `None`; in that case the trajectory's own
        `start_state` (populated by the planner that produced it) is used.

        Returns `(frames, polyline)`. `polyline` is `None` when there are
        fewer than two points. Any failure along the way returns `([], None)`
        rather than raising.
        """
        start_state = start_state or self.start_state

        if robot_cell is None:
            return ([], None)
        if self.points is None:
            return ([], None)
        if start_state is None:
            return ([], None)

        # ROS `trajectory_msgs/JointTrajectoryPoint` carries no joint_names,
        # names live on the parent `JointTrajectory`. Fall back accordingly.
        point_names = self.points[0].joint_names or self.joint_names or []

        # One copy outside the loop; per-point we only swap the configuration.
        working_state = start_state.copy()
        base = working_state.robot_configuration

        # Pre-compute merge data once. If we can't merge by name, fall back to
        # using the point directly — works when the trajectory and cell share
        # the same joint set.
        merge_data = None
        if base is not None and point_names:
            base_names = list(base.joint_names)
            merge_data = (
                base_names,
                list(base.joint_types),
                list(base.joint_values),
                {name: idx for idx, name in enumerate(base_names)},
            )

        frames = []
        try:
            for point in self.points:
                if merge_data is None:
                    working_state.robot_configuration = point
                else:
                    base_names, base_types, base_values_template, name_to_pos = merge_data
                    values = list(base_values_template)
                    for name, value in zip(point_names, point.joint_values):
                        idx = name_to_pos.get(name)
                        if idx is not None:
                            values[idx] = value
                    working_state.robot_configuration = Configuration(values, base_types, base_names)
                frames.append(
                    robot_cell.forward_kinematics_target_frame(
                        robot_cell_state=working_state,
                        target_mode=TargetMode.ROBOT,
                        group=group or None,
                    )
                )
        except Exception:
            return ([], None)

        polyline = Polyline([f.point for f in frames]) if len(frames) >= 2 else None
        return (frames, polyline)
