import pytest

from compas_fab.robots import Configuration
from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint


@pytest.fixture
def trj():
    p1 = JointTrajectoryPoint([1.571, 0, 0, 0.262, 0, 0],
                              [0] * 6,
                              [3.] * 6,
                              time_from_start=Duration(2, 1293))
    p2 = JointTrajectoryPoint([0.571, 0, 0, 0.262, 0, 0],
                              [0] * 6,
                              [3.] * 6,
                              time_from_start=Duration(6, 0))
    config = Configuration.from_revolute_values([0.] * 6)

    return JointTrajectory(trajectory_points=[p1, p2],
                           joint_names=['joint_1', 'joint_2', 'joint_3',
                                        'joint_4', 'joint_5', 'joint_6'],
                           start_configuration=config)


def test_trajectory_points(trj):
    assert(trj.time_from_start == Duration(6, 0).seconds)
    assert(len(trj.points) == 2)


def test_serialization(trj):
    data = trj.to_data()
    new_trj = JointTrajectory.from_data(data)
    assert(new_trj.to_data() == data)
