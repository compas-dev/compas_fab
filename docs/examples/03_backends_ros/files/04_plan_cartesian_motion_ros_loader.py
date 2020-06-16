from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots import Configuration

with RosClient() as client:
    robot = client.load_robot()
    group = robot.main_group_name

    frames = []
    frames.append(Frame((0.2925, 0.3949, 0.5066), (0, 1, 0), (0, 0, 1)))
    frames.append(Frame((0.5103, 0.2827, 0.4074), (0, 1, 0), (0, 0, 1)))

    start_configuration = Configuration.from_revolute_values((0.667, -0.298, 0.336, -2.333, -1.787, 2.123, 0.571))
    options = {
        'max_step': 0.01,
        'avoid_collisions': True,
    }

    trajectory = robot.plan_cartesian_motion(frames,
                                             start_configuration,
                                             group=group,
                                             options=options)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
