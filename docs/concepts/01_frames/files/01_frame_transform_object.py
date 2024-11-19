from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Frame
from compas.geometry import Transformation

point = Point(146.00, 150.00, 161.50)
xaxis = Vector(0.9767, 0.0010, -0.214)
yaxis = Vector(0.1002, 0.8818, 0.4609)

# Coordinate system represented by Frame F
F = Frame(point, xaxis, yaxis)

# A point in F's local coordinate system
P = Point(35.0, 35.0, 35.0)

# Computing the point's position in the global (world) coordinates
P_ = F.to_world_coordinates(P)
print(P_)  # >>> Point(x=190.314, y=164.389, z=200.285)

# This is equivalent to the following transformation
T_WCF_LCF = Transformation.from_frame(F)
P_ = P.transformed(T_WCF_LCF)
print(P_)  # >>> Point(x=190.314, y=164.389, z=200.285)
