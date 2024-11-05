from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    planner = MoveItPlanner(client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [0, -1, 0], [0, 0, -1])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    start_state = robot_cell.default_cell_state()

    result_count = 0
    for config in planner.iter_inverse_kinematics(target, start_state, options=dict(max_results=10)):
        print("Found configuration", config)
        result_count += 1
    print("Found %d configurations" % result_count)
