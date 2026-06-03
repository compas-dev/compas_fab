"""Compose multi-stage motions into a serializable MotionPlan.

A MotionPlan threads a single RobotCellState through a chain of
trajectories and discrete state changes. The typical use case is pick-
and-place: plan an approach, plan the contact motion, record the grasp
(state change), plan the retract — then save the whole thing.

This example plans three cartesian motions on a UR5 with a cone tool,
drops a discrete "grasp" state change between two of them, assembles a
MotionPlan, and roundtrips it through JSON.
"""

from compas.geometry import Frame

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import MotionPlan
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    robot_cell, start_state = RobotCellLibrary.ur5_cone_tool()
    planner.set_robot_cell(robot_cell)

    # Start configuration reachable to the waypoints below.
    start_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    # 1. Cartesian descend — TCP drops straight to the grasp pose.
    descend = planner.plan_cartesian_motion(
        FrameWaypoints(
            [Frame([0.4, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])],
            target_mode=TargetMode.TOOL,
        ),
        start_state,
    )

    # 2. Cartesian retract — TCP lifts back up to clear the part.
    state_at_grasp = start_state.copy()
    state_at_grasp.robot_configuration.merge(descend.points[-1])
    retract = planner.plan_cartesian_motion(
        FrameWaypoints(
            [Frame([0.4, 0.1, 0.7], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])],
            target_mode=TargetMode.TOOL,
        ),
        state_at_grasp,
    )

    # 3. Cartesian transfer — move laterally above the drop-off, then descend.
    state_after_retract = state_at_grasp.copy()
    state_after_retract.robot_configuration.merge(retract.points[-1])
    transfer = planner.plan_cartesian_motion(
        FrameWaypoints(
            [
                Frame([0.4, -0.1, 0.7], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
                Frame([0.4, -0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ],
            target_mode=TargetMode.TOOL,
        ),
        state_after_retract,
    )

    # Assemble the plan. State changes (here: grasp, release) sit BETWEEN
    # trajectories; they carry an explicit post-state. Trajectory steps
    # derive their post-state automatically from the chain.
    #
    # In a real workflow `state_after_grasp` and `state_after_release` would
    # also reflect the grip — e.g. attaching a rigid body to the tool via
    # RigidBodyState, then detaching it. Here we keep it minimal and just
    # carry the running configuration.
    state_after_grasp = state_at_grasp.copy()
    state_after_release = state_after_retract.copy()
    state_after_release.robot_configuration.merge(transfer.points[-1])

    plan = (
        MotionPlan(name="pick_and_place", start_state=start_state, robot_cell=robot_cell, description="UR5 cone tool demo")
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

    # Roundtrip through JSON. The plan owns the chain so each contained
    # trajectory's start_state is stripped on serialize and reattached on
    # load — no per-trajectory state duplication. Use compas's JSON helpers
    # (not stdlib json) since states contain compas.geometry.Frame etc.
    encoded = plan.to_jsonstring()
    print("\nSerialised size: {} KB".format(len(encoded) // 1024))
    plan.to_json("pick_and_place_plan.json", pretty=True)  # also save to file for manual inspection

    loaded = MotionPlan.from_jsonstring(encoded)
    loaded.verify_cell(robot_cell)  # raises ValueError on cell mismatch
    print("Roundtripped, cell verified.")
    print("End-state robot configuration:", loaded.end_state.robot_configuration.joint_values)
