from compas.geometry import Vector

from compas_fab.robots import Wrench


def test_serialization():
    w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
    nw = Wrench.__from_data__(w.__data__)
    assert nw.force == [1, 2, 3]
    assert nw.torque == [0.1, 0.2, 0.3]

    w = Wrench(Vector(1, 2, 3), Vector(0.1, 0.2, 0.3))
    nw = Wrench.__from_data__(w.__data__)
    assert nw.force == Vector(1, 2, 3)
    assert nw.torque == Vector(0.1, 0.2, 0.3)
