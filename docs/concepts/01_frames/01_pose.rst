.. _pose:

*******************************************************************************
Pose
*******************************************************************************

.. currentmodule:: compas.geometry

The :class:`Frame` is a fundamental geometry object in the
**COMPAS** framework that are used to describe the pose of physical rigid bodies
(such as robot links and collision objects) and non-physical entities (such as
coordinate systems and targets). The :class:`Frame` consists of a point and and two
orthonormal base vectors (xaxis, yaxis), which are the minimal information required
to define a coordinate system in 3D space relative to another coordinate system.
The frame represents all six degrees of freedom (3 translations and 3 rotations)
of a rigid body in 3D space and thus can be said to fully describe the pose
(position and orientation) of a rigid body.

Note that **COMPAS FAB** only supports right-handed coordinate systems. This is
the same convention used by most industrial robot manufacturers and the Robot
Operating System (ROS).

.. literalinclude :: files/01_frame_constructor.py
   :language: python

Transformations
===============

A pose is often defined as a frame with respect to another frame (a base frame).
In this regard, the frame representation is equivalent to a rigid affine
transformation (a combination of translation and rotation) that maps one
coordinate system to another. If the user is more comfortable with the
transformation representation for defining poses, the :class:`Transformation`
class can be used, which can be converted to a :class:`Frame` and vice versa.
See the documentation for :class:`Frame`, :class:`Transformation`,
:class:`Translation` and :class:`Rotation` for a comprehensive list of constructors,
properties, and methods.

.. literalinclude :: files/01_frame_conversion.py
   :language: python


The following example shows how a frame is used to represent a coordinate system
relative to the world coordinate system.
A point ``P`` in the local (user-defined, relative) coordinate system of
frame ``F``. The position of ``P`` in the global (world, absolute) coordinate system
can be obtained by the method :meth:`Frame.to_world_coordinates`.

.. literalinclude :: files/01_frame_transform_object.py
   :language: python


Other Representations
=====================

Industrial robots do not have a common way of describing the pose orientation.
The :class:`Frame` class provides different methods to convert orientation information
to various conventions.

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
    alpha, beta, gamma = F1.euler_angles(False, 'xyz')

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

