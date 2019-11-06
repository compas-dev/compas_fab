from compas.geometry import Frame
from compas.robots import RobotModel

from compas_fab.backends import RosClient
from compas_fab.backends import RosFileServerLoader
from compas_fab.robots import Configuration
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics

with RosClient() as client:
    loader = RosFileServerLoader(client)

    urdf = loader.load_urdf()
    srdf = loader.load_srdf()

    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)

    robot = Robot(model, semantics=semantics, client=client)
    group = robot.main_group_name

    frames = []
    frames.append(Frame((0.2925, 0.3949, 0.5066), (0, 1, 0), (0, 0, 1)))
    frames.append(Frame((0.5103, 0.2827, 0.4074), (0, 1, 0), (0, 0, 1)))

    start_configuration = Configuration.from_revolute_values((0.667, -0.298, 0.336, -2.333, -1.787, 2.123, 0.571))

    trajectory = robot.plan_cartesian_motion(frames,
                                             start_configuration,
                                             max_step=0.01,
                                             avoid_collisions=True,
                                             group=group)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
