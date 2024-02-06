import math

from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Rotation
from compas.geometry import Vector

from compas_fab.utilities import arange


class OrthonormalVectorsFromAxisGenerator(object):
    """Generate vectors that are orthonormal to an axis.

    Parameters
    ----------
    axis : :class:`compas.geometry.Vector`
        The axis to which the vectors should be orthonormal to.
    angle_step : float
        The angle in radians as the spacing between generated vectors.

    Yields
    ------
    :class:`compas.geometry.Vector`
        The orthonormal vector.

    Examples
    --------
    >>> generator = OrthonormalVectorsFromAxisGenerator((0, 0, 1), math.radians(120))
    >>> [print(xaxis) for xaxis in generator]
    Vector(x=0.000, y=-1.000, z=0.000)
    Vector(x=0.866, y=0.500, z=0.000)
    Vector(x=-0.866, y=0.500, z=0.000)
    [None, None, None]
    """

    def __init__(self, axis, angle_step, start_vector=None):
        self.axis = axis
        self.angle_step = angle_step
        self.start_vector = start_vector

    @property
    def start_vector(self):
        return self._start_vector

    @start_vector.setter
    def start_vector(self, start_vector):
        self._start_vector = Vector(*list(start_vector)) if start_vector else None

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, axis):
        self._axis = Vector(*list(axis))

    def __iter__(self):
        if self.start_vector:
            # correct start_vector
            self.start_vector = (self.axis.cross(self.start_vector.unitized())).cross(self.axis)
            for alpha in arange(0, 2 * math.pi, self.angle_step):
                R = Rotation.from_axis_and_angle(self.axis, alpha)
                yield self.start_vector.transformed(R)
        else:
            f = Frame.from_plane(Plane((0, 0, 0), self.axis))
            for alpha in arange(0, 2 * math.pi, self.angle_step):
                x = math.cos(alpha)
                y = math.sin(alpha)
                yield f.to_world_coordinates(Vector(x, y, 0))


class DeviationVectorsGenerator(object):
    """Calculates equally distributed vectors that deviate from the specified one by a maximal angle of `max_alpha`.

    Parameters
    ----------
    axis : :class:`compas.geometry.Vector`
        The axis about which to calculate vectors.
    max_alpha : float
        The maximum angle in radians that a vector should deviate from the axis.
    step : int
        The number of how often to divide `max_angle`.

    Yields
    ------
    list of :class:`compas.geometry.Vector`

    Examples
    --------
    >>> generator = DeviationVectorsGenerator((0, 0, -1), math.radians(120), 1)
    >>> [print(zaxis) for zaxis in generator]
    Vector(x=0.000, y=0.000, z=-1.000)
    Vector(x=-0.866, y=0.000, z=0.500)
    Vector(x=0.433, y=0.750, z=0.500)
    Vector(x=0.433, y=-0.750, z=0.500)
    [None, None, None, None]
    """

    def __init__(self, axis, max_alpha, step):
        self.axis = axis
        self.max_alpha = float(max_alpha)
        self.step = int(step)

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, axis):
        self._axis = Vector(*list(axis))

    def __iter__(self):
        yield self.axis
        alphas = arange(
            self.max_alpha / self.step, self.max_alpha + self.max_alpha / self.step, self.max_alpha / self.step
        )
        radii = [math.sin(alpha) for alpha in alphas]
        x, y = math.cos(alphas[0]), math.sin(alphas[0])
        d = math.sqrt((1 - x) ** 2 + y**2)
        # get any vector normal to axis
        axis2 = Frame.from_plane(Plane((0, 0, 0), self.axis)).xaxis
        for alpha, r in zip(alphas, radii):
            R1 = Rotation.from_axis_and_angle(axis2, alpha)
            amount = int(round(2 * math.pi * r / d))
            betas = arange(0, 2 * math.pi, 2 * math.pi / amount)
            for beta in betas:
                R2 = Rotation.from_axis_and_angle(self.axis, beta)
                yield self.axis.transformed(R2 * R1)
