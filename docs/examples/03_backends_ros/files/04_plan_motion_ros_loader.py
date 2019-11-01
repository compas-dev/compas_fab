import math
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

    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    tolerance_position = 0.001
    tolerance_axes = [math.radians(1)] * 3

    start_configuration = Configuration.from_revolute_values(
        (0.667, -0.298, 0.336, -2.333, -1.787, 2.123, 0.571))

    # create goal constraints from frame
    goal_constraints = robot.constraints_from_frame(frame,
                                                    tolerance_position,
                                                    tolerance_axes,
                                                    group)

    trajectory = robot.plan_motion(goal_constraints,
                                   start_configuration,
                                   group,
                                   planner_id='RRT')

    print("Computed kinematic path with %d configurations." % len(trajectory.points))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
