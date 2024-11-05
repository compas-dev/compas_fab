import math

from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)
    assert robot_cell.robot_model.name == "ur5_robot"

    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    tolerance_position = 0.001
    tolerance_orientation = math.radians(1)

    start_state = robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.530, 3.830, -0.580, -3.330, 4.760, 0.000]
    group = robot_cell.main_group_name

    # create target from frame
    target = FrameTarget(
        frame,
        TargetMode.ROBOT,
        tolerance_position,
        tolerance_orientation,
    )

    options = {
        "planner_id": "RRTConnect",
        # "max_velocity_scaling_factor": 0.01,
        # "max_acceleration_scaling_factor": 0.01,
    }
    trajectory = planner.plan_motion(target, start_state, group, options=options)

    print("Computed kinematic path with %d configurations." % len(trajectory.points))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
