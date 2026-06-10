"""Cartesian interpolation helpers shared by Cartesian motion planners.

These helpers are backend-agnostic — they only depend on `compas.geometry` — and
compute evenly-spaced interpolation steps between two poses (or two point-axis
pairs), honouring a maximum Cartesian and angular step. `FrameInterpolator` can
additionally report whether a segment may still be subdivided (used by planners
that subdivide when a joint jump is too large).
"""

from math import ceil

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Quaternion
from compas.geometry import Vector
from compas.geometry import axis_angle_from_quaternion
from compas.geometry import cross_vectors
from compas.geometry import is_parallel_vector_vector

__all__ = [
    "FrameInterpolator",
    "PointAxisInterpolator",
]


class FrameInterpolator:
    """Interpolate between two frames with bounded Cartesian and angular steps.

    All the properties are read-only and are computed during initialization.

    Parameters
    ----------
    start_frame : [`Frame`][compas.geometry.Frame]
        The start frame.
    end_frame : [`Frame`][compas.geometry.Frame]
        The end frame.
    max_step_distance : float, optional
        The maximum Cartesian distance between two consecutive interpolated frames,
        in the frames' own unit. Default is ``0.01``.
    max_step_angle : float, optional
        The maximum angular distance (radians) between two consecutive interpolated
        frames. Default is ``0.1``.
    min_step_distance : float, optional
        The minimum Cartesian distance below which a segment can no longer be
        subdivided (see [`check_if_subdivision_possible`][compas_fab.robots.FrameInterpolator.check_if_subdivision_possible]).
        Default is ``0.0001``.
    min_step_angle : float, optional
        The minimum angular distance (radians) below which a segment can no longer
        be subdivided. Default is ``0.0001``.

    """

    def __init__(
        self,
        start_frame: Frame,
        end_frame: Frame,
        max_step_distance: float = 0.01,
        max_step_angle: float = 0.1,
        min_step_distance: float = 0.0001,
        min_step_angle: float = 0.0001,
    ):
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.max_step_distance = max_step_distance
        self.max_step_angle = max_step_angle
        self.min_step_distance = min_step_distance
        self.min_step_angle = min_step_angle

        # Compute the total distance between the two frames
        self._total_distance = self.start_frame.point.distance_to_point(self.end_frame.point)

        # Compute the total angle between the two frames
        delta_frame = self.start_frame.to_local_coordinates(self.end_frame)
        _, self._total_angle = axis_angle_from_quaternion(Quaternion.from_frame(delta_frame))

        # Compute the number of steps based on max_step_distance and max_step_angle
        num_steps_by_distance = ceil(self._total_distance / self.max_step_distance)
        num_steps_by_angle = ceil(self._total_angle / self.max_step_angle)
        # NOTE: Minimum of 1 step is used, this step is equal to the end_frame itself.
        self._regular_interpolation_steps = max(num_steps_by_distance, num_steps_by_angle, 1)

    @property
    def total_distance(self) -> float:
        """The total distance between the start and end frames.

        Returns
        -------
        float
            The total distance (original unit) between the start and end frames.

        """
        return self._total_distance

    @property
    def total_angle(self) -> float:
        """The total angle between the start and end frames.

        Returns
        -------
        float
            The total angle in radians between the start and end frames.

        """
        return self._total_angle

    @property
    def regular_interpolation_steps(self) -> int:
        """The number of interpolation steps based on max_step_distance and max_step_angle.

        Returns
        -------
        int
            The number of interpolation steps.
            Minimum is 1.

        """

        return self._regular_interpolation_steps

    def get_interpolated_frame(self, t: float) -> Frame:
        """Interpolate between two frames using a parameter t.

        Parameters
        ----------
        t : float
            The interpolation parameter, 0 <= t <= 1.

        Returns
        -------
        [`Frame`][compas.geometry.Frame]
            The interpolated frame.

        """
        # Compute the interpolated frame
        current_frame = self.start_frame.interpolate_frame(self.end_frame, t)
        return current_frame

    def check_if_subdivision_possible(self, t1: float, t2: float) -> tuple:
        """Check if the addition of an extra t value between t1 and t2 is possible.

        Two conditions are being checked:

        1. The distance between the two frames is greater than the `min_step_distance`.
        2. The angle between the two frames is greater than the `min_step_angle`.

        The subdivision is possible if any one of the two conditions is met.

        Parameters
        ----------
        t1 : float
            The start parameter.
        t2 : float
            The end parameter.

        Returns
        -------
        tuple
            A tuple containing the following values:

            - ``delta_distance``: (:obj:`float`) The distance between the two frames.
            - ``delta_angle``: (:obj:`float`) The angle between the two frames.
            - ``subdivision_possible``: (:obj:`bool`) True if the subdivision is possible
        """
        delta_t = abs(t2 - t1)

        delta_distance = self.total_distance * delta_t
        delta_angle = self.total_angle * delta_t

        if delta_distance < self.min_step_distance * 2 and delta_angle < self.min_step_angle * 2:
            return (delta_distance, delta_angle, False)

        return (delta_distance, delta_angle, True)


class PointAxisInterpolator:
    """Interpolate between two point-axis pairs with bounded position/angle steps.

    All the properties are read-only and are computed during initialization.

    Parameters
    ----------
    start_point_axis : tuple of [`Point`][compas.geometry.Point] and [`Vector`][compas.geometry.Vector]
        The start point-axis pair.
    end_point_axis : tuple of [`Point`][compas.geometry.Point] and [`Vector`][compas.geometry.Vector]
        The end point-axis pair.
    max_step_distance : float, optional
        The maximum Cartesian distance between two consecutive interpolated points,
        in the points' own unit. Default is ``0.01``.
    max_step_angle : float, optional
        The maximum angular distance (radians) between two consecutive interpolated
        axes. Default is ``0.1``.

    """

    def __init__(
        self,
        start_point_axis: tuple[Point, Vector],
        end_point_axis: tuple[Point, Vector],
        max_step_distance: float = 0.01,
        max_step_angle: float = 0.1,
    ):
        self.start_point, self.start_axis = start_point_axis
        self.end_point, self.end_axis = end_point_axis
        self.max_step_distance = max_step_distance
        self.max_step_angle = max_step_angle

        # Compute the total distance between the two points
        self._total_distance = self.start_point.distance_to_point(self.end_point)

        # Compute the total angle between the two axes
        self._total_angle = self.start_axis.angle(self.end_axis)

        # Compute the number of steps based on max_step_distance and max_step_angle
        num_steps_by_distance = ceil(self._total_distance / self.max_step_distance)
        num_steps_by_angle = ceil(self._total_angle / self.max_step_angle)

        # NOTE: Minimum of 1 step is used, this step is equal to the end point-axis itself.
        self._regular_interpolation_steps = max(num_steps_by_distance, num_steps_by_angle, 1)

    @property
    def total_distance(self) -> float:
        """The total distance between the start and end points.

        Returns
        -------
        float
            The total distance (original unit) between the start and end points.

        """
        return self._total_distance

    @property
    def total_angle(self) -> float:
        """The total angle between the start and end axes.

        Returns
        -------
        float
            The total angle in radians between the start and end axes.

        """
        return self._total_angle

    @property
    def regular_interpolation_steps(self) -> int:
        """The number of interpolation steps based on max_step_distance and max_step_angle.

        Returns
        -------
        int
            The number of interpolation steps.
            Minimum is 1.

        """

        return self._regular_interpolation_steps

    def get_interpolated_point_axis(self, t: float) -> tuple:
        """Interpolate between two point-axis pairs using a parameter t.

        Parameters
        ----------
        t : float
            The interpolation parameter, 0 <= t <= 1.

        Returns
        -------
        tuple of [`Point`][compas.geometry.Point] and [`Vector`][compas.geometry.Vector]
            The interpolated point-axis pair.

        """
        # Compute the interpolated point using the parameter t
        xyz = [a + t * (b - a) for a, b in zip(self.start_point, self.end_point)]
        point_t = xyz
        # Compute the interpolated axis by rotation
        if is_parallel_vector_vector(self.start_axis, self.end_axis):
            axis_t = self.end_axis
        else:
            angle = t * self.total_angle
            rotation_axis = cross_vectors(self.start_axis, self.end_axis)
            axis_t = self.start_axis.rotated(angle, axis=rotation_axis)

        return (point_t, axis_t)
