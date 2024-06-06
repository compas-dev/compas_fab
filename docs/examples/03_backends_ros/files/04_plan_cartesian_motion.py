from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.robots import FrameWaypoints

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == "ur5_robot"

    frames = []
    frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
    frames.append(Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0]))
    waypoints = FrameWaypoints(frames)

    start_configuration = robot.zero_configuration()
    start_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)
    options = {
        "max_step": 0.01,
        "avoid_collisions": True,
    }

    trajectory = robot.plan_cartesian_motion(waypoints, start_configuration, options=options)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
