import math
import time

from compas.geometry import Translation

from compas_fab.backends import PyRokiPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

robot_cell, state = RobotCellLibrary.ur5(load_geometry=False)
planner = PyRokiPlanner()
planner.set_robot_cell(robot_cell)

center = robot_cell.robot_model.forward_kinematics(
    state.robot_configuration,
    robot_cell.get_end_effector_link_name(),
)

for step in range(60):
    angle = 2.0 * math.pi * step / 60.0
    offset = [0.005 * math.cos(angle), 0.005 * math.sin(angle), 0.0]
    target_frame = center.transformed(Translation.from_vector(offset))

    started = time.perf_counter()
    configuration = planner.inverse_kinematics(
        FrameTarget(target_frame, TargetMode.ROBOT),
        state,
        options={"check_collision": False},
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0

    # Feed the result into the next call. Keeping the planner and updating the
    # seed are both required to benefit from the compiled problem cache.
    state.robot_configuration = state.robot_configuration.merged(configuration)
    print("Step {:02d}: {:7.2f} ms".format(step, elapsed_ms))
