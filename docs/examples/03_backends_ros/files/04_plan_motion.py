import math

from compas.geometry import Frame

from compas_fab.backends import RosClient

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == 'ur5_robot'

    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    tolerance_position = 0.001
    tolerance_axes = [math.radians(1)] * 3

    start_configuration = robot.zero_configuration()
    start_configuration.joint_values = (-3.530, 3.830, -0.580, -3.330, 4.760, 0.000)
    group = robot.main_group_name

    # create goal constraints from frame
    goal_constraints = robot.constraints_from_frame(frame,
                                                    tolerance_position,
                                                    tolerance_axes,
                                                    group)

    trajectory = robot.plan_motion(goal_constraints,
                                   start_configuration,
                                   group,
                                   options=dict(
                                       planner_id='RRTConnect'
                                   ))

    print("Computed kinematic path with %d configurations." % len(trajectory.points))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
