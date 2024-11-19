from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCellLibrary

with RosClient() as client:
    planner = MoveItPlanner(client)
    # Loading RobotCell from RobotCellLibrary with tool and workpiece
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam(client)

    # Load RobotCell from ROS MoveIt backend and assert that the robot model is the same
    client.load_robot_cell(False)
    assert client.robot_cell.root_name == robot_cell.root_name

    # Set the tool and workpiece in the ROS MoveIt backend
    planner.set_robot_cell(robot_cell)

    # The default robot cell state already have the gripper and beam attached
    # We modify the robot's configuration here to a specific joint configuration
    # The OCF is = Frame([0.35, 0.1, 0.5], [1, 0, 0], [0, -1, 0])
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    start_frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    print("Workpiece's OCF at the starting state:\n    {}".format(start_frame))

    # The following waypoints will make the object move in a square
    frames = []
    frames.append(Frame([0.35, 0.3, 0.5], [1, 0, 0], [0, -1, 0]))
    frames.append(Frame([0.35, 0.3, 0.7], [1, 0, 0], [0, -1, 0]))
    frames.append(Frame([0.35, 0.1, 0.7], [1, 0, 0], [0, -1, 0]))
    frames.append(start_frame)  # Goes back to the starting position
    waypoints = FrameWaypoints(frames, TargetMode.WORKPIECE)

    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

"""
Output: (may vary)
>>> Workpiece's OCF at the starting state:
>>>     Frame(point=Point(x=0.350, y=0.100, z=0.500), xaxis=Vector(x=1.000, y=-0.000, z=-0.000), yaxis=Vector(x=-0.000, y=-1.000, z=0.000))
>>> Computed cartesian path with 84 configurations,
>>> following 100% of requested trajectory.
>>> Executing this path at full speed would take approx. 6.012 seconds.
"""
