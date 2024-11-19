from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Frame

point = Point(10.0, 20.0, 30.0)
xaxis = Vector(1, 0, 0)
yaxis = Vector(0, 1, 0)

F = Frame(point, xaxis, yaxis)
print(F)

# Alternative constructor without importing Point and Vector
F = Frame([10.0, 20.0, 30.0], [1, 0, 0], [0, 1, 0])
print(F)

# Default frames
F = Frame.worldXY()
print(F)
