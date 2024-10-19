from compas_robots import Configuration
from compas_fab.robots import RobotCellLibrary

robot_cell, robot_cell_state = RobotCellLibrary.ur5()
configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])

frame_WCF = robot_cell.robot_model.forward_kinematics(configuration)

print("Frame in the world coordinate system")
print(str(frame_WCF.point))

assert str(frame_WCF.point) == "Point(x=0.300, y=0.100, z=0.500)"
