import pytest

from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import MotionPlan
from compas_fab.robots import PlanStep
from compas_fab.robots import RobotCellLibrary


def _make_trajectory(joint_names, points, total_seconds):
    """Build a trajectory with N points sharing the given joint_names."""
    step_seconds = total_seconds / max(len(points) - 1, 1)
    tps = []
    for i, values in enumerate(points):
        tps.append(
            JointTrajectoryPoint(
                joint_values=list(values),
                joint_types=[0] * len(values),
                time_from_start=Duration(int(i * step_seconds), 0),
                joint_names=list(joint_names),
            )
        )
    return JointTrajectory(trajectory_points=tps, joint_names=list(joint_names))


@pytest.fixture
def cell_and_state():
    return RobotCellLibrary.ur5(load_geometry=False)


@pytest.fixture
def joint_names(cell_and_state):
    cell, state = cell_and_state
    return list(state.robot_configuration.joint_names)


@pytest.fixture
def traj1(joint_names):
    return _make_trajectory(joint_names, [[0.0] * 6, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]], total_seconds=2.0)


@pytest.fixture
def traj2(joint_names):
    return _make_trajectory(joint_names, [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]], total_seconds=3.0)


def test_empty_plan(cell_and_state):
    cell, state = cell_and_state
    plan = MotionPlan(name="empty", start_state=state)
    assert len(plan) == 0
    assert plan.end_state is state
    assert plan.duration == 0.0
    assert plan.trajectories == []


def test_append_trajectory_derives_post_state(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="single", start_state=state).append_trajectory("move", traj1)

    assert len(plan) == 1
    assert plan.duration == pytest.approx(2.0)
    assert plan.trajectories == [traj1]

    end = plan.end_state
    # Configuration updated to the trajectory's last point, joint by joint
    assert end.robot_configuration.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    # start_state must remain pristine
    assert state.robot_configuration.joint_values != end.robot_configuration.joint_values


def test_chained_pick_and_place(cell_and_state, traj1, traj2):
    cell, state = cell_and_state
    grasped_state = state.copy()
    released_state = state.copy()

    plan = (
        MotionPlan(name="pnp", start_state=state)
        .append_trajectory("approach_pick", traj1)
        .append_state_change("grasp", grasped_state)
        .append_trajectory("retract", traj2)
        .append_state_change("release", released_state)
    )

    assert len(plan) == 4
    names = [s.name for s in plan]
    assert names == ["approach_pick", "grasp", "retract", "release"]
    assert plan.duration == pytest.approx(5.0)
    # end_state of the plan == post_state of the last step == released_state
    assert plan.end_state is released_state


def test_duplicate_step_name_raises(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="dup", start_state=state).append_trajectory("move", traj1)
    with pytest.raises(ValueError, match="already used"):
        plan.append_trajectory("move", traj1)


def test_trajectory_joint_validation(cell_and_state):
    cell, state = cell_and_state
    bogus = _make_trajectory(["not_a_real_joint"], [[0.0], [1.0]], total_seconds=1.0)
    plan = MotionPlan(name="bad", start_state=state)
    with pytest.raises(ValueError, match="not present in the cell state"):
        plan.append_trajectory("oops", bogus)


def test_step_by_name(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="lookup", start_state=state).append_trajectory("move", traj1)
    step = plan.step_by_name("move")
    assert step.is_trajectory
    assert step.duration == pytest.approx(2.0)
    with pytest.raises(KeyError, match="No step named"):
        plan.step_by_name("missing")


def test_cell_signature_match(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="bound", start_state=state, robot_cell=cell).append_trajectory("move", traj1)
    assert plan.cell_signature is not None
    plan.verify_cell(cell)  # no raise


def test_cell_signature_mismatch_raises(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="bound", start_state=state, robot_cell=cell).append_trajectory("move", traj1)
    other_cell, _ = RobotCellLibrary.panda(load_geometry=False)
    with pytest.raises(ValueError, match="different cell"):
        plan.verify_cell(other_cell)


def test_verify_cell_noop_when_unbound(cell_and_state):
    cell, state = cell_and_state
    plan = MotionPlan(name="unbound", start_state=state)
    assert plan.cell_signature is None
    plan.verify_cell(cell)  # no raise


def test_append_step_dispatcher(cell_and_state, traj1):
    cell, state = cell_and_state
    plan = MotionPlan(name="dispatch", start_state=state)
    plan.append_step(PlanStep(name="move", trajectory=traj1, description="d"))
    grasped = state.copy()
    plan.append_step(PlanStep(name="grasp", post_state=grasped))
    assert [s.name for s in plan] == ["move", "grasp"]
    assert plan.step_by_name("move").is_trajectory
    assert plan.step_by_name("move").description == "d"
    assert plan.step_by_name("grasp").post_state is grasped


def test_append_step_state_change_without_post_state_raises(cell_and_state):
    cell, state = cell_and_state
    plan = MotionPlan(name="bad", start_state=state)
    with pytest.raises(ValueError, match="requires an explicit post_state"):
        plan.append_step(PlanStep(name="grasp"))


def test_serialization_roundtrip(cell_and_state, traj1, traj2):
    cell, state = cell_and_state
    grasped_state = state.copy()

    plan = (
        MotionPlan(name="pnp", start_state=state, robot_cell=cell, description="hi")
        .append_trajectory("approach", traj1, description="approach the pickup")
        .append_state_change("grasp", grasped_state)
        .append_trajectory("retract", traj2)
    )

    data = plan.__data__
    # start_state should be stripped from trajectory blobs to avoid duplication
    assert data["steps"][0]["trajectory"]["start_state"] is None
    assert data["steps"][2]["trajectory"]["start_state"] is None
    # state-change steps must serialize their post_state
    assert data["steps"][1]["post_state"] is not None

    loaded = MotionPlan.__from_data__(data)
    assert loaded.name == "pnp"
    assert loaded.description == "hi"
    assert loaded.cell_signature == plan.cell_signature
    assert [s.name for s in loaded] == ["approach", "grasp", "retract"]
    assert loaded.duration == pytest.approx(plan.duration)

    # trajectories have their start_state restored from the chain
    approach = loaded.step_by_name("approach")
    assert approach.trajectory is not None
    assert approach.trajectory.start_state is not None
    # the restored start_state matches the plan's start_state for the first step
    assert approach.trajectory.start_state.robot_configuration.joint_values == \
        loaded.start_state.robot_configuration.joint_values

    # second trajectory's start_state == grasped_state (the post_state preceding it)
    retract = loaded.step_by_name("retract")
    assert retract.trajectory.start_state.robot_configuration.joint_values == \
        loaded.step_by_name("grasp").post_state.robot_configuration.joint_values
