from math import pi

from compas.robots import Joint
from compas_fab.robots import Configuration
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import to_degrees


def test_revolute_ctor():
    q = [4.5, 1.7, 0.5, 2.1, 0.1, 2.1]
    config = Configuration.from_revolute_values(q)
    assert config.types == [Joint.REVOLUTE] * 6

    config = Configuration.from_revolute_values([pi/2, 0., 0.])
    assert to_degrees(config.joint_values) == [90, 0, 0]


def test_prismatic_revolute_ctor():
    config = Configuration.from_prismatic_and_revolute_values(
        [8.312], [1.5, 0., 0., 0., 1., 0.8])
    assert config.joint_values == [8.312, 1.5, 0., 0., 0., 1., 0.8]
    assert config.types == [Joint.PRISMATIC] + [Joint.REVOLUTE] * 6


def test_ctor():
    values = [1., 3., 0.1]
    types = [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR]
    config = Configuration(values, types)
    assert config.joint_values == values
    assert config.types == types


def test_scale():
    values = [1.5, 1., 2.]
    types = [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR]
    config = Configuration(values, types)
    config.scale(1000.)

    assert config.joint_values == [1.5, 1000., 2000.]


def test_cast_to_str():
    config = Configuration([pi/2, 3., 0.1], [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR])
    assert str(config) == 'Configuration((1.571, 3.000, 0.100), (0, 2, 5))'


def test_from_data():
    config = Configuration.from_data(dict(joint_values=[8.312, 1.5],
                                          types=[Joint.PRISMATIC, Joint.REVOLUTE]))
    assert str(config) == 'Configuration((8.312, 1.500), (2, 0))'


def test_to_data():
    config = Configuration.from_prismatic_and_revolute_values(
        [8.312], [1.5, 0., 0., 0., 1., 0.8])
    # types=[Joint.PRISMATIC, Joint.REVOLUTE]))
    data = config.to_data()

    assert data['joint_values'] == [8.312, 1.5, 0., 0., 0., 1., 0.8]
    assert data['types'] == [Joint.PRISMATIC] + [Joint.REVOLUTE] * 6


def test_config_merged():
    config = Configuration(joint_values=[1, 2, 3], types=[Joint.REVOLUTE]*3, joint_names=['a', 'b', 'c'])
    other_config = Configuration(joint_values=[3, 2, 0], types=[Joint.REVOLUTE]*3, joint_names=['a', 'b', 'd'])
    new_config = config.merged(other_config)
    assert new_config.joint_dict == {'a': 3, 'b': 2, 'c': 3, 'd': 0}


def test_joint_trajectory_point_merged():
    tjp = JointTrajectoryPoint(joint_values=[1, 2, 3], types=[Joint.REVOLUTE]*3, velocities=[4, 5, 6])
    tjp.joint_names = ['a', 'b', 'c']
    other_tjp = JointTrajectoryPoint(joint_values=[3, 2, 0], types=[Joint.REVOLUTE]*3, velocities=[0, 5, 0])
    other_tjp.joint_names = ['a', 'b', 'd']
    new_tjp = tjp.merged(other_tjp)
    assert new_tjp.joint_dict == {'a': 3, 'b': 2, 'c': 3, 'd': 0}
    assert new_tjp.velocity_dict == {'a': 0, 'b': 5, 'c': 6, 'd': 0}
