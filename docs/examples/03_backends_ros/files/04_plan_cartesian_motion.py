from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

with RosClient() as client:
    planner = MoveItPlanner(client)
    robot_cell = client.load_robot_cell()
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

    assert robot_cell.robot_model.name == "ur5_robot"

    frames = []
    frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
    frames.append(Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0]))
    waypoints = FrameWaypoints(frames, TargetMode.ROBOT)

    robot_cell_state.robot_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)
    options = {
        "max_step": 0.01,
        "avoid_collisions": True,
    }

    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
