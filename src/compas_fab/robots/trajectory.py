from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.robots.time_ import Duration
from compas_fab.robots.configuration import Configuration

__all__ = [
    'JointTrajectoryPoint',
    'JointTrajectory',
    'Trajectory',
]


class JointTrajectoryPoint(Configuration):
    """Defines a point within a trajectory.

    A trajectory point is a sub-class of :class:`Configuration` extended
    with acceleration, effort and time from start information.

    Trajectory points are defined either as *values + velocities and
    accelerations*, or as *values + efforts*.

    Attributes
    ----------
    values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective type.
    types : :obj:`list` of :class:`compas.robots.Joint.TYPE`
        Joint types, e.g. a list of :class:`compas.robots.Joint.REVOLUTE` for revolute joints.
    velocities : :obj:`list` of :obj:`float`
        Velocity of each joint.
    accelerations : :obj:`list` of :obj:`float`
        Acceleration of each joint.
    effort : :obj:`list` of :obj:`float`
        Effort of each joint.
    time_from_start : :class:`Duration`
        Duration of trajectory point counting from the start.
    """

    def __init__(self, values=[], types=[], velocities=[], accelerations=[], effort=[], time_from_start=None):
        super(JointTrajectoryPoint, self).__init__(values, types)
        self.velocities = velocities or len(values) * [0.]
        self.accelerations = accelerations or len(values) * [0.]
        self.effort = effort or len(values) * [0.]
        self.time_from_start = time_from_start or Duration(0, 0)

    def __str__(self):
        vs = '%.' + self._precision
        return 'JointTrajectoryPoint(({}), {}, ({}), ({}), ({}), {})'.format(
            ', '.join(vs % i for i in self.values),
            tuple(self.types),
            ', '.join(vs % i for i in self.velocities),
            ', '.join(vs % i for i in self.accelerations),
            ', '.join(vs % i for i in self.effort),
            self.time_from_start,
        )

    @property
    def positions(self):
        """Alias of ``values``."""
        return self.values

    @property
    def velocities(self):
        return self._velocities

    @velocities.setter
    def velocities(self, velocities):
        if len(self.values) != len(velocities):
            raise ValueError('Must have {} velocities, but {} given.'.format(
                len(self.values), len(velocities)))

        self._velocities = velocities

    @property
    def accelerations(self):
        return self._accelerations

    @accelerations.setter
    def accelerations(self, accelerations):
        if len(self.values) != len(accelerations):
            raise ValueError('Must have {} accelerations, but {} given.'.format(
                len(self.values), len(accelerations)))

        self._accelerations = accelerations

    @property
    def effort(self):
        return self._effort

    @effort.setter
    def effort(self, effort):
        if len(self.values) != len(effort):
            raise ValueError('Must have {} efforts, but {} given.'.format(
                len(self.values), len(effort)))

        self._effort = effort

    @property
    def data(self):
        """:obj:`dict` : The data representing the trajectory point.

        By assigning a data dictionary to this property, the current data of the
        configuration will be replaced by the data in the dict. The data getter
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
        self.values = data.get('values') or []
        self.types = data.get('types') or []
        self.velocities = data.get('velocities') or []
        self.accelerations = data.get('accelerations') or []
        self.effort = data.get('effort') or []
        self.time_from_start = Duration.from_data(data.get('time_from_start') or {})


class Trajectory(object):
    """Base trajectory class.

    Attributes
    ----------
    planning_time: :obj:`float`
        Amount of time it took to complete the motion plan
    """

    def __init__(self):
        self.planning_time = None


class JointTrajectory(Trajectory):
    """Describes a joint trajectory as a list of trajectory points.

    Attributes
    ----------
    trajectory_points: :obj:`list` of :class:`JointTrajectoryPoint`
        List of points composing the trajectory.
    joint_names: :obj:`list` of :obj:`str`
        List of joint names of the trajectory.
    start_configuration: :class:`Configuration`
        Start configuration for the trajectory.
    fraction: float
        Indicates the percentage of requested trajectory that was calcuted,
        e.g. ``1`` means the full trajectory was found.
    """

    def __init__(self, trajectory_points=[], joint_names=[], start_configuration=None, fraction=None):
        super(Trajectory, self).__init__()
        self.points = trajectory_points
        self.joint_names = joint_names
        self.start_configuration = start_configuration
        self.fraction = fraction

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
        """Return the data dictionary that represents the trajectory, and from
        which it can be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the trajectory."""
        data_obj = {}
        data_obj['points'] = [p.to_data() for p in self.points]
        data_obj['joint_names'] = self.joint_names or []
        data_obj['start_configuration'] = self.start_configuration.to_data()
        data_obj['fraction'] = self.fraction

        return data_obj

    @data.setter
    def data(self, data):
        self.points = list(map(JointTrajectoryPoint.from_data, data.get('points') or []))
        self.joint_names = data.get('joint_names', [])
        self.start_configuration = Configuration.from_data(data.get('start_configuration'))
        self.fraction = data.get('fraction')

    @property
    def time_from_start(self):
        """Effectively, time from start for the last point in the trajectory.
        """
        if not self.points:
            return 0.

        return self.points[-1].time_from_start.seconds


if __name__ == '__main__':
    p1 = JointTrajectoryPoint([1.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.] * 6)
    p1.time_from_start = Duration(2, 1293)

    p2 = JointTrajectoryPoint([0.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.] * 6)
    p2.time_from_start = Duration(6, 0)

    trj = JointTrajectory([p1, p2])
    print(trj)
    # p.accelerations = [2,3,4,3,5,1]
    # p.time_from_start = [Duration(1, 23)]
    # print(p1.to_data())
    # print(JointTrajectoryPoint.from_data(p1.to_data()))

    print(trj.points)
    print(trj.to_data())
    print(JointTrajectory.from_data(trj.to_data()).to_data())
