"""Classes used to define robot trajectories."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots.time_ import Duration
from compas_fab.robots.configuration import Configuration
from compas_fab.robots.configuration import FixedLengthList

__all__ = [
    'JointTrajectoryPoint',
    'JointTrajectory',
    'Trajectory',
]


class JointTrajectoryPoint(Configuration):
    """Defines a point within a trajectory.

    A trajectory point is a sub-class of :class:`Configuration` extended
    with acceleration, effort and time from start information.

    Trajectory points are defined either as *joint_values + velocities and
    accelerations*, or as *joint_values + efforts*.

    Parameters
    ----------
    joint_values : :obj:`list` of :obj:`float`, optional
        Joint values expressed in radians or meters, depending on the respective
        type.
    joint_types : :obj:`list` of :attr:`compas.robots.Joint.TYPE`, optional
        Joint types, e.g. a :obj:`list` of
        :attr:`compas.robots.Joint.REVOLUTE` for revolute joints.
    velocities : :obj:`list` of :obj:`float`, optional
        Velocity of each joint.
    accelerations : :obj:`list` of :obj:`float`, optional
        Acceleration of each joint.
    effort : :obj:`list` of :obj:`float`, optional
        Effort of each joint.
    time_from_start : :class:`Duration`, optional
        Duration of trajectory point counting from the start.

    Attributes
    ----------
    joint_values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective
        type.
    joint_types : :obj:`list` of :attr:`compas.robots.Joint.TYPE`
        Joint types, e.g. a :obj:`list` of
        :attr:`compas.robots.Joint.REVOLUTE` for revolute joints.
    velocities : :obj:`list` of :obj:`float`
        Velocity of each joint.
    accelerations : :obj:`list` of :obj:`float`
        Acceleration of each joint.
    effort : :obj:`list` of :obj:`float`
        Effort of each joint.
    time_from_start : :class:`Duration`
        Duration of trajectory point counting from the start.
    positions : :obj:`list` of :obj:`float`
        Alias of `joint_values`.
    data : obj:`dict`
        The data representing the trajectory point.
    """

    def __init__(self, joint_values=None, joint_types=None, velocities=None, accelerations=None, effort=None, time_from_start=None, joint_names=None):
        super(JointTrajectoryPoint, self).__init__(joint_values, joint_types, joint_names)
        self.velocities = velocities or len(self.joint_values) * [0.]
        self.accelerations = accelerations or len(self.joint_values) * [0.]
        self.effort = effort or len(self.joint_values) * [0.]
        self.time_from_start = time_from_start or Duration(0, 0)

    def __str__(self):
        """Return a human-readable string representation of the instance."""
        vs = '%.' + self._precision
        return 'JointTrajectoryPoint(({}), {}, ({}), ({}), ({}), {})'.format(
            ', '.join(vs % i for i in self.joint_values),
            tuple(self.joint_types),
            ', '.join(vs % i for i in self.velocities),
            ', '.join(vs % i for i in self.accelerations),
            ', '.join(vs % i for i in self.effort),
            self.time_from_start,
        )

    @property
    def positions(self):
        """:obj:`list` of :obj:`float` : Alias of `joint_values`."""
        return self.joint_values

    @property
    def velocities(self):
        """:obj:`list` of :obj:`float` : Velocity of each joint."""
        return self._velocities

    @velocities.setter
    def velocities(self, velocities):
        if len(self.joint_values) != len(velocities):
            raise ValueError('Must have {} velocities, but {} given.'.format(
                len(self.joint_values), len(velocities)))

        self._velocities = FixedLengthList(velocities)

    @property
    def accelerations(self):
        """:obj:`list` of :obj:`float` : Acceleration of each joint."""
        return self._accelerations

    @accelerations.setter
    def accelerations(self, accelerations):
        if len(self.joint_values) != len(accelerations):
            raise ValueError('Must have {} accelerations, but {} given.'.format(
                len(self.joint_values), len(accelerations)))

        self._accelerations = FixedLengthList(accelerations)

    @property
    def effort(self):
        """:obj:`list` of :obj:`float` : Effort of each joint."""
        return self._effort

    @effort.setter
    def effort(self, effort):
        if len(self.joint_values) != len(effort):
            raise ValueError('Must have {} efforts, but {} given.'.format(
                len(self.joint_values), len(effort)))

        self._effort = FixedLengthList(effort)

    @property
    def data(self):
        """:obj:`dict` : The data representing the trajectory point.

        By assigning a data dictionary to this property, the current data of the
        configuration will be replaced by the data in the :obj:`dict`. The data getter
        and setter should always be used in combination with each other.
        """
        data_obj = super(JointTrajectoryPoint, self).data
        data_obj['velocities'] = self.velocities
        data_obj['accelerations'] = self.accelerations
        data_obj['effort'] = self.effort
        data_obj['time_from_start'] = self.time_from_start.to_data()

        return data_obj

    @data.setter
    def data(self, data):
        self._joint_values = FixedLengthList(data.get('joint_values') or [])
        self._joint_types = FixedLengthList(data.get('joint_types') or [])
        self._velocities = FixedLengthList(data.get('velocities') or [])
        self._accelerations = FixedLengthList(data.get('accelerations') or [])
        self._effort = FixedLengthList(data.get('effort') or [])
        self.time_from_start = Duration.from_data(data.get('time_from_start') or {})

    def copy(self):
        """Create a copy of this :class:`JointTrajectoryPoint`.

        Returns
        -------
        :class:`JointTrajectoryPoint`
            An instance of :class:`JointTrajectoryPoint`
        """
        cls = type(self)
        return cls.from_data(self.data)

    @property
    def velocity_dict(self):
        """A dictionary of joint velocities by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.velocities))

    @property
    def acceleration_dict(self):
        """A dictionary of joint accelerations by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.accelerations))

    @property
    def effort_dict(self):
        """A dictionary of joint efforts by joint name."""
        self.check_joint_names()
        return dict(zip(self.joint_names, self.effort))

    def merged(self, other):
        """Get a new ``JointTrajectoryPoint`` with this ``JointTrajectoryPoint`` merged
        with another ``JointTrajectoryPoint``.  The other ``JointTrajectoryPoint``
        takes precedence over this ``JointTrajectoryPoint`` in case a joint value is present in both.

        Note
        ----
            Caution: ``joint_names`` may be rearranged.

        Parameters
        ----------
        other : :class:`JointTrajectoryPoint`
            The ``JointTrajectoryPoint`` to be merged.

        Returns
        -------
        :class:`JointTrajectoryPoint`
            A ``JointTrajectoryPoint`` with values for all included joints.

        Raises
        ------
        :exc:`ValueError`
            If the ``JointTrajectoryPoint`` or the other ``JointTrajectoryPoint`` does not specify
            joint names for all joint values.
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


class Trajectory(object):
    """Base trajectory class.

    Attribute
    ---------
    planning_time : :obj:`float`
        Amount of time it took to complete the motion plan
    """

    def __init__(self):
        self.planning_time = None


class JointTrajectory(Trajectory):
    """Describes a joint trajectory as a list of trajectory points.

    Parameters
    ----------
    trajectory_points : :obj:`list` of :class:`JointTrajectoryPoint`, optional
        List of points composing the trajectory.
    joint_names : :obj:`list` of :obj:`str`, optional
        List of joint names of the trajectory.
    start_configuration : :class:`Configuration`, optional
        Start configuration for the trajectory.
    fraction : :obj:`float`, optional
        Indicates the percentage of requested trajectory that was calculated,
        e.g. ``1`` means the full trajectory was found.
    attached_collision_meshes : :obj:`list` of :closs:`compas_fab.robots.AttachedCollisionMesh`
        The attached collision meshes included in the calculation of this trajectory.

    Attributes
    ----------
    points : :obj:`list` of :class:`JointTrajectoryPoint`
        List of points composing the trajectory.
    joint_names : :obj:`list` of :obj:`str`
        List of joint names of the trajectory.
    start_configuration : :class:`Configuration`
        Start configuration for the trajectory.
    fraction : :obj:`float`
        Indicates the percentage of requested trajectory that was calculated,
        e.g. ``1`` means the full trajectory was found.
    attached_collision_meshes : :obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`
        The attached collision meshes included in the calculation of this trajectory.
    data : :obj:`dict`
        The data representing the trajectory.
    """

    def __init__(self, trajectory_points=None, joint_names=None, start_configuration=None, fraction=None, attached_collision_meshes=None):
        super(Trajectory, self).__init__()
        self.points = trajectory_points or []
        self.joint_names = joint_names or []
        self.start_configuration = start_configuration
        self.fraction = fraction
        self.attached_collision_meshes = attached_collision_meshes or []

    @classmethod
    def from_data(cls, data):
        """Construct a trajectory from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`JointTrajectory`
             An instance of :class:`JointTrajectory`.
        """
        trajectory = cls()
        trajectory.data = data
        return trajectory

    def to_data(self):
        """Get the data dictionary that represents the trajectory.

        This can be used to reconstruct the :class:`Trajectory` instance.

        Returns
        -------
        :obj:`dict`
        """
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the trajectory."""
        data_obj = {}
        data_obj['points'] = [p.to_data() for p in self.points]
        data_obj['joint_names'] = self.joint_names or []
        data_obj['start_configuration'] = self.start_configuration.to_data() if self.start_configuration else None
        data_obj['fraction'] = self.fraction
        data_obj['attached_collision_meshes'] = [acm.to_data() for acm in self.attached_collision_meshes]

        return data_obj

    @data.setter
    def data(self, data):
        self.points = list(map(JointTrajectoryPoint.from_data, data.get('points') or []))
        self.joint_names = data.get('joint_names', [])
        if data.get('start_configuration'):
            self.start_configuration = Configuration.from_data(data.get('start_configuration'))
        self.fraction = data.get('fraction')
        self.attached_collision_meshes = [AttachedCollisionMesh.from_data(acm_data) for acm_data in data.get('attached_collision_meshes')]

    @property
    def time_from_start(self):
        """:obj:`float` : Effectively, time from start for the last point in the trajectory."""
        if not self.points:
            return 0.

        return self.points[-1].time_from_start.seconds
