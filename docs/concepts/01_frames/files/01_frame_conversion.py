from compas.geometry import Frame
from compas.geometry import Transformation

# Convert Frame to Transformation
F = Frame.worldXY()
T = Transformation.from_frame(F)
print(T)

# Conversion to Frame
T = Transformation([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]])
F = Frame.from_transformation(T)
print(F)
