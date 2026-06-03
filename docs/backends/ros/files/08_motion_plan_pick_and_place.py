"""Compose multi-stage motions into a serializable MotionPlan, MoveIt edition.

Mirrors the PyBullet example: plans three cartesian motions on a UR5,
intersperses two discrete state changes (grasp / release), assembles a
MotionPlan, and roundtrips it through JSON.

Requires a running rosbridge serving MoveIt 1 or MoveIt 2.
Pass `port=` to RosClient as needed.
"""

from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import MotionPlan
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"

    start_state = robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)

    # 1. Cartesian descend — TCP drops toward the grasp pose.
    descend = planner.plan_cartesian_motion(
        FrameWaypoints([Frame([0.3, 0.1, 0.4], [1, 0, 0], [0, 1, 0])], TargetMode.ROBOT),
        start_state,
    )

    # 2. Cartesian retract — TCP lifts back up.
    state_at_grasp = start_state.copy()
    state_at_grasp.robot_configuration = state_at_grasp.robot_configuration.merged(descend.points[-1])
    retract = planner.plan_cartesian_motion(
        FrameWaypoints([Frame([0.3, 0.1, 0.6], [1, 0, 0], [0, 1, 0])], TargetMode.ROBOT),
        state_at_grasp,
    )

    # 3. Cartesian transfer — move laterally above the drop-off, then descend.
    state_after_retract = state_at_grasp.copy()
    state_after_retract.robot_configuration = state_after_retract.robot_configuration.merged(retract.points[-1])
    transfer = planner.plan_cartesian_motion(
        FrameWaypoints(
            [
                Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0]),
                Frame([0.5, 0.1, 0.4], [1, 0, 0], [0, 1, 0]),
            ],
            TargetMode.ROBOT,
        ),
        state_after_retract,
    )

    # Assemble. State changes (grasp, release) carry an explicit post-state.
    # In a real workflow they would also reflect the grip — e.g. attaching a
    # rigid body to the tool — but here we just carry the running config.
    state_after_grasp = state_at_grasp.copy()
    state_after_release = state_after_retract.copy()
    state_after_release.robot_configuration = state_after_release.robot_configuration.merged(transfer.points[-1])

    plan = (
        MotionPlan(name="pick_and_place", start_state=start_state, robot_cell=robot_cell, description="UR5 MoveIt demo")
        .append_trajectory("descend", descend, description="cartesian descent to pickup")
        .append_state_change("grasp", state_after_grasp, description="gripper closes")
        .append_trajectory("retract", retract, description="cartesian retract")
        .append_trajectory("transfer", transfer, description="lateral + descend to placement")
        .append_state_change("release", state_after_release, description="gripper opens")
    )

    print("Plan {!r} ({})".format(plan.name, plan.description))
    print("  cell signature: {}...".format(plan.cell_signature[:16]))
    print("  steps:          {}".format(len(plan)))
    print("  trajectories:   {}".format(len(plan.trajectories)))
    for step in plan:
        kind = "trajectory" if step.is_trajectory else "state change"
        points = "{} pts".format(len(step.trajectory.points)) if step.is_trajectory else ""
        print("    - {:<10} {:<12} {}".format(step.name, kind, points))

    # Roundtrip via compas's JSON helpers — `to_jsonstring` handles compas
    # geometry types (Frame, Configuration, etc.) that stdlib json doesn't.
    encoded = plan.to_jsonstring()
    print("\nSerialised size: {} KB".format(len(encoded) // 1024))

    loaded = MotionPlan.from_jsonstring(encoded)
    loaded.verify_cell(robot_cell)  # raises ValueError on cell mismatch
    print("Roundtripped, cell verified.")
    print("End-state robot configuration:", loaded.end_state.robot_configuration.joint_values)
