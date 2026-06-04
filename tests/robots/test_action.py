import pytest

from compas_fab.robots import Action
from compas_fab.robots import ActionChain
from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
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


def test_empty_chain(cell_and_state):
    cell, state = cell_and_state
    chain = ActionChain(name="empty", start_state=state)
    assert len(chain) == 0
    assert chain.end_state is state
    assert chain.duration == 0.0
    assert chain.trajectories == []


def test_append_trajectory_derives_post_state(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="single", start_state=state).append_trajectory("move", traj1)

    assert len(chain) == 1
    assert chain.duration == pytest.approx(2.0)
    assert chain.trajectories == [traj1]

    end = chain.end_state
    # Configuration updated to the trajectory's last point, joint by joint
    assert end.robot_configuration.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    # start_state must remain pristine
    assert state.robot_configuration.joint_values != end.robot_configuration.joint_values


def test_append_trajectory_mirrors_start_state(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="mirror", start_state=state).append_trajectory("move", traj1)
    action = chain.action_by_name("move")
    # Action owns the start state; the trajectory mirrors it
    assert action.start_state is state
    assert action.trajectory.start_state is state


def test_chained_pick_and_place(cell_and_state, traj1, traj2):
    cell, state = cell_and_state
    grasped_state = state.copy()
    released_state = state.copy()

    chain = (
        ActionChain(name="pnp", start_state=state)
        .append_trajectory("approach_pick", traj1)
        .append_state_change("grasp", grasped_state)
        .append_trajectory("retract", traj2)
        .append_state_change("release", released_state)
    )

    assert len(chain) == 4
    names = [a.name for a in chain]
    assert names == ["approach_pick", "grasp", "retract", "release"]
    assert chain.duration == pytest.approx(5.0)
    # end_state of the chain == post_state of the last action == released_state
    assert chain.end_state is released_state


def test_duplicate_action_name_raises(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="dup", start_state=state).append_trajectory("move", traj1)
    with pytest.raises(ValueError, match="already used"):
        chain.append_trajectory("move", traj1)


def test_trajectory_joint_validation(cell_and_state):
    cell, state = cell_and_state
    bogus = _make_trajectory(["not_a_real_joint"], [[0.0], [1.0]], total_seconds=1.0)
    chain = ActionChain(name="bad", start_state=state)
    with pytest.raises(ValueError, match="not present in the cell state"):
        chain.append_trajectory("oops", bogus)


def test_action_by_name(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="lookup", start_state=state).append_trajectory("move", traj1)
    action = chain.action_by_name("move")
    assert action.is_trajectory
    assert action.is_planned
    assert action.duration == pytest.approx(2.0)
    with pytest.raises(KeyError, match="No action named"):
        chain.action_by_name("missing")


def test_tags_default_and_assignment(cell_and_state, traj1):
    cell, state = cell_and_state
    # No tags -> empty list by default, stored under attributes["tags"]
    plain = Action(name="plain", trajectory=traj1)
    assert plain.tags == []
    assert plain.attributes["tags"] == []

    # tags constructor arg lands in attributes and round-trips through the property
    tagged = Action(name="tagged", trajectory=traj1, tags=["approach", "linear"])
    assert tagged.tags == ["approach", "linear"]
    assert tagged.attributes["tags"] == ["approach", "linear"]

    tagged.tags = ["retract"]
    assert tagged.attributes["tags"] == ["retract"]


def test_tags_preserved_through_chain(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="tags", start_state=state).append_trajectory(
        "move", traj1, tags=["approach", "linear"]
    )
    assert chain.action_by_name("move").tags == ["approach", "linear"]


def test_cell_signature_match(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="bound", start_state=state, robot_cell=cell).append_trajectory("move", traj1)
    assert chain.cell_signature is not None
    chain.verify_cell(cell)  # no raise


def test_cell_signature_mismatch_raises(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="bound", start_state=state, robot_cell=cell).append_trajectory("move", traj1)
    other_cell, _ = RobotCellLibrary.panda(load_geometry=False)
    with pytest.raises(ValueError, match="different cell"):
        chain.verify_cell(other_cell)


def test_verify_cell_noop_when_unbound(cell_and_state):
    cell, state = cell_and_state
    chain = ActionChain(name="unbound", start_state=state)
    assert chain.cell_signature is None
    chain.verify_cell(cell)  # no raise


def test_append_action_dispatcher(cell_and_state, traj1):
    cell, state = cell_and_state
    chain = ActionChain(name="dispatch", start_state=state)
    chain.append_action(Action(name="move", trajectory=traj1, description="d", tags=["linear"]))
    grasped = state.copy()
    chain.append_action(Action(name="grasp", post_state=grasped))
    assert [a.name for a in chain] == ["move", "grasp"]
    move = chain.action_by_name("move")
    assert move.is_trajectory
    assert move.description == "d"
    assert move.tags == ["linear"]
    assert chain.action_by_name("grasp").post_state is grasped


def test_append_action_state_change_without_post_state_raises(cell_and_state):
    cell, state = cell_and_state
    chain = ActionChain(name="bad", start_state=state)
    with pytest.raises(ValueError, match="requires an explicit post_state"):
        chain.append_action(Action(name="grasp"))


def test_iter_cell_states_covers_every_point_and_state_change(cell_and_state, traj1, traj2):
    cell, state = cell_and_state
    grasped = state.copy()
    chain = (
        ActionChain(name="iter", start_state=state)
        .append_trajectory("descend", traj1)
        .append_state_change("grasp", grasped)
        .append_trajectory("retract", traj2)
    )
    states = list(chain.iter_cell_states())
    # 2 (traj1) + 1 (state change) + 2 (traj2)
    assert len(states) == 5
    # First snapshot matches traj1.points[0] applied to start_state
    assert states[0].robot_configuration.joint_values == [0.0] * 6
    # Snapshot after the first trajectory matches traj1's last point
    assert states[1].robot_configuration.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    # State-change snapshot is a copy of `grasped`
    assert states[2] is not grasped
    assert states[2].robot_configuration.joint_values == grasped.robot_configuration.joint_values
    # Final snapshot matches traj2's last point
    assert states[-1].robot_configuration.joint_values == [1.0] * 6
    # Snapshots are independent — mutating one does not affect the chain or others
    states[0].robot_configuration.joint_values = [9.9] * 6
    assert chain.start_state.robot_configuration.joint_values != [9.9] * 6
    assert states[1].robot_configuration.joint_values != [9.9] * 6


def test_iter_cell_states_empty_chain(cell_and_state):
    cell, state = cell_and_state
    chain = ActionChain(name="empty", start_state=state)
    assert list(chain.iter_cell_states()) == []


def test_serialization_roundtrip(cell_and_state, traj1, traj2):
    cell, state = cell_and_state
    grasped_state = state.copy()

    chain = (
        ActionChain(name="pnp", start_state=state, robot_cell=cell, description="hi")
        .append_trajectory("approach", traj1, description="approach the pickup", tags=["approach", "linear"])
        .append_state_change("grasp", grasped_state)
        .append_trajectory("retract", traj2)
    )

    data = chain.__data__
    # start_state should be stripped from trajectory blobs to avoid duplication
    assert data["actions"][0]["trajectory"]["start_state"] is None
    assert data["actions"][2]["trajectory"]["start_state"] is None
    # state-change actions must serialize their post_state
    assert data["actions"][1]["post_state"] is not None
    # tags ride along in the attributes bag
    assert data["actions"][0]["attributes"]["tags"] == ["approach", "linear"]

    loaded = ActionChain.__from_data__(data)
    assert loaded.name == "pnp"
    assert loaded.description == "hi"
    assert loaded.cell_signature == chain.cell_signature
    assert [a.name for a in loaded] == ["approach", "grasp", "retract"]
    assert loaded.duration == pytest.approx(chain.duration)
    assert loaded.action_by_name("approach").tags == ["approach", "linear"]

    # trajectories have their start_state restored from the chain
    approach = loaded.action_by_name("approach")
    assert approach.trajectory is not None
    assert approach.trajectory.start_state is not None
    # the restored start_state matches the chain's start_state for the first action
    assert approach.trajectory.start_state.robot_configuration.joint_values == \
        loaded.start_state.robot_configuration.joint_values

    # second trajectory's start_state == grasped_state (the post_state preceding it)
    retract = loaded.action_by_name("retract")
    assert retract.trajectory.start_state.robot_configuration.joint_values == \
        loaded.action_by_name("grasp").post_state.robot_configuration.joint_values
