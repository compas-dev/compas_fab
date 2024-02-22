import pytest
from compas_robots import Configuration
from compas_robots.model import Joint

from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint


@pytest.fixture
def jtp():
    return JointTrajectoryPoint(
        [1.571, 0, 0, 0.262, 0, 0],
        [0] * 6,
        [3.0] * 6,
        time_from_start=Duration(2, 1293),
        joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    )


@pytest.fixture
def trj():
    p1 = JointTrajectoryPoint([1.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.0] * 6, time_from_start=Duration(2, 1293))
    p2 = JointTrajectoryPoint([0.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.0] * 6, time_from_start=Duration(6, 0))
    config = Configuration.from_revolute_values([0.0] * 6)

    trajectory = JointTrajectory(
        trajectory_points=[p1, p2],
        joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        start_configuration=config,
    )
    trajectory.planning_time = 0.5
    return trajectory


def test_trajectory_points(trj):
    assert trj.time_from_start == Duration(6, 0).seconds
    assert len(trj.points) == 2


def test_joint_trajectory_point_serialization(jtp):
    data = jtp.__data__
    new_jtp = JointTrajectoryPoint.__from_data__(data)
    assert new_jtp.__data__ == data
    assert new_jtp["joint_1"] == 1.571


def test_serialization(trj):
    data = trj.__data__
    new_trj = JointTrajectory.__from_data__(data)
    assert new_trj.__data__ == data
    assert new_trj.planning_time == 0.5
    assert new_trj.time_from_start == Duration(6, 0).seconds


def test_joint_trajectory_point_merged():
    tjp = JointTrajectoryPoint(joint_values=[1, 2, 3], joint_types=[Joint.REVOLUTE] * 3, velocities=[4, 5, 6])
    tjp.joint_names = ["a", "b", "c"]
    other_tjp = JointTrajectoryPoint(joint_values=[3, 2, 0], joint_types=[Joint.REVOLUTE] * 3, velocities=[0, 5, 0])
    other_tjp.joint_names = ["a", "b", "d"]
    new_tjp = tjp.merged(other_tjp)
    assert new_tjp.joint_dict == {"a": 3, "b": 2, "c": 3, "d": 0}
    assert new_tjp.velocity_dict == {"a": 0, "b": 5, "c": 6, "d": 0}
