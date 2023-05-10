"""
Visualizes a trajectory.

COMPAS FAB v0.28.0
"""
from compas_ghpython import draw_frame
from compas_ghpython import list_to_ghtree
from ghpythonlib.componentbase import executingcomponent as component


class TrajectoryVisualize(component):
    def RunScript(self, robot, group, trajectory):
        start_configuration = None
        configurations = []
        fraction = 0.0
        time = 0.0

        planes = []
        positions = []
        velocities = []
        accelerations = []

        if robot and trajectory:
            group = group or robot.main_group_name

            for c in trajectory.points:
                configurations.append(
                    robot.merge_group_with_full_configuration(c, trajectory.start_configuration, group)
                )
                frame = robot.forward_kinematics(c, group, options=dict(solver="model"))
                planes.append(draw_frame(frame))
                positions.append(c.positions)
                velocities.append(c.velocities)
                accelerations.append(c.accelerations)

            start_configuration = trajectory.start_configuration
            fraction = trajectory.fraction
            time = trajectory.time_from_start

        P = list_to_ghtree(list(zip(*positions)))
        V = list_to_ghtree(list(zip(*velocities)))
        A = list_to_ghtree(list(zip(*accelerations)))

        # return outputs if you have them; here I try it for you:
        return (start_configuration, configurations, fraction, time, planes, P, V, A)
