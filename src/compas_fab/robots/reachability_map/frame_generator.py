import math
from compas.geometry import Vector
from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Rotation


def arange(start, stop, step, tol=1e-6):
    """Return evenly spaced values within a given interval.

    Values are generated within the half-open interval [start, stop) (in other
    words, the interval including start but excluding stop). Similar to numpy's
    arange function.
    """
    assert(step != 0)
    values = []
    while start < stop and stop - start > tol:
        values.append(start)
        start += step
    return values


class OrthonormalVectorsFromAxis(object):
    """Generate vectors that are orthonormal an axis.

    Parameters
    ----------
    axis : Vector
        The axis to which the vectors should be orthonormal to
    angle_step : float
        The angle in radians as the spacing between generated vectors.

    Yields
    ------
    Vector
        The orthonormal vector.

    Examples
    --------
    >>> generator = OrthonormalVectorsFromAxis((0, 0, 1), math.radians(120))
    >>> xaxes = [xaxis for xaxis in generator]
    >>> print(xaxes)
    [Vector(0.000, -1.000, 0.000), Vector(0.866, 0.500, 0.000), Vector(-0.866, 0.500, 0.000)]
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
            for alpha in arange(0, 2*math.pi, self.angle_step):
                R = Rotation.from_axis_and_angle(self.axis, alpha)
                yield self.start_vector.transformed(R)
        else:
            f = Frame.from_plane(Plane((0, 0, 0), self.axis))
            for alpha in arange(0, 2*math.pi, self.angle_step):
                x = math.cos(alpha)
                y = math.sin(alpha)
                yield f.to_world_coordinates(Vector(x, y, 0))


class Deviation_vectors_generator
    def deviation_vectors_generator(cls, axis, max_alpha, step):
        """Calculates equally distributed vectors that deviate from the passed one by a maximal angle of max_alpha.

        Parameters
        ----------
        axis : Vector
            The axis about which to calculate vectors.
        max_alpha : float
            The maximum angle in radians that a vector should deviate from the axis.
        step : int
            The number of how often to divide `max_angle`.

        Yields
        ------
        list of Vectors

        Examples
        --------
        >>>
        """
        axis = Vector(*axis)
        yield axis
        alphas = arange(max_alpha/step, max_alpha + max_alpha/step, max_alpha/step)
        radii = [math.sin(alpha) for alpha in alphas]
        x, y = math.cos(alphas[0]), math.sin(alphas[0])
        d = math.sqrt((1 - x)**2 + y**2)
        # get any vector normal to axis
        axis2 = Frame.from_plane(Plane((0, 0, 0), axis)).xaxis
        for alpha, r in zip(alphas, radii):
            R1 = Rotation.from_axis_and_angle(axis2, alpha)
            amount = int(round(2*math.pi*r/d))
            betas = arange(0, 2*math.pi, 2*math.pi/amount)
            for beta in betas:
                R2 = Rotation.from_axis_and_angle(axis, beta)
                yield axis.transformed(R2 * R1)
    

class FrameGenerator(object):
    """A frame generator generates frames for the reachability map
    """
    # Define your own frame generator, this is specific to the application, but
    # class methods from FrameGenerator may help. # TODO: move somewhere else?
    # Frame generators should yield a number and a frame. # TODO: make class?

    def frame_generator(self, pt):
        counter = 0
        for zaxis in self.deviation_vectors_generator((0, 0, -1), math.radians(90), 1):
            for xaxis in self.orthonormal_vectors_from_axis_generator(zaxis, math.radians(90)):
                yaxis = zaxis.cross(xaxis)
                yield counter, Frame(pt, xaxis, yaxis)
                counter += 1
    
    def __next__(self):
        return_value = self.a
        self.a, self.b = self.b, self.a+self.b
        return return_value

    def __iter__(self):
        return self

    


if __name__ == "__main__":

    generator = OrthonormalVectorsFromAxis((0, 0, 1), math.radians(120))
    xaxes = [xaxis for xaxis in generator]
    print(xaxes)