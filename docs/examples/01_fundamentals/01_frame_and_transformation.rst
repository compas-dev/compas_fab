*******************************************************************************
Frame and Transformation
*******************************************************************************

.. currentmodule:: compas.geometry

:class:`Frame` and :class:`Transformation` are two basic classes in the
**COMPAS** framework and can be used to describe position/orientation and
coordinate systems. The :class:`Frame` consists of a point and and two
orthonormal base vectors (xaxis, yaxis). :class:`Transformation` is the base
class for transformations like :class:`Rotation`, :class:`Translation`,
:class:`Scale`, :class:`Reflection`, :class:`Projection` and :class:`Shear`.

Here is a simple example of how to use a frame as a coordinate system: Starting
from a point ``P`` in the local (user-defined, relative) coordinate system of
frame ``F``, i.e. its position is relative to the origin and orientation of
``F``, we want to get the position of ``P`` in the global (world, absolute)
coordinate system.

.. code-block:: python

    from compas.geometry import Point
    from compas.geometry import Vector
    from compas.geometry import Frame

    point = Point(146.00, 150.00, 161.50)
    xaxis = Vector(0.9767, 0.0010, -0.214)
    yaxis = Vector(0.1002, 0.8818, 0.4609)

    # coordinate system F
    F = Frame(point, xaxis, yaxis)

    # point in F (local coordinates)
    P = Point(35., 35., 35.)
    # point in global (world) coordinates
    P_ = F.to_world_coords(P)


Industrial robots do not have a common way of describing the pose orientation.
The frame provides methods to specify the orientation in various conventions.

.. code-block:: python

    from compas.geometry import Point
    from compas.geometry import Vector
    from compas.geometry import Frame

    point = Point(146.00, 150.00, 161.50)
    xaxis = Vector(0.9767, 0.0010, -0.214)
    yaxis = Vector(0.1002, 0.8818, 0.4609)

    F = Frame(point, xaxis, yaxis)

    print(F.quaternion) # ABB
    print(F.euler_angles(static=False, axes='xyz')) # Staubli
    print(F.euler_angles(static=False, axes='zyx')) # KUKA
    print(F.axis_angle_vector) # UR


From the orientation (:class:`Rotation`)
of the frame, several other representations of rotation can be derived, such as
Euler angles, axis-angle representation, and quaternion.

.. code-block:: python

    from compas.geometry import Frame
    from compas.geometry import Rotation

    F1 = Frame([0, 0, 0], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])

    # euler angles
    args = False, 'xyz'
    alpha, beta, gamma = F1.euler_angles(*args)

    # check if angles are correct
    xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
    Rx = Rotation.from_axis_and_angle(xaxis, alpha)
    Ry = Rotation.from_axis_and_angle(yaxis, beta)
    Rz = Rotation.from_axis_and_angle(zaxis, gamma)
    F2 = Frame.worldXY()
    print('Are equal?', F1 == F2.transformed(Rx * Ry * Rz))

    # quaternion
    q = F1.quaternion
    F2 = Frame.from_quaternion(q)
    print('Are equal?', F1 == F2)

    # axis-angle
    ax = F1.axis_angle_vector
    F2 = Frame.from_axis_angle_vector(ax)
    print('Are equal?', F1 == F2)


Further information
===================

* `Rotation <https://en.wikipedia.org/wiki/Rotation>`_
* `Euler angles <https://en.wikipedia.org/wiki/Euler_angles>`_
* `Quaternion <https://en.wikipedia.org/wiki/Quaternion>`_
* `Axis–angle representation <https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation>`_

