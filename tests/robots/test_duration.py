from compas_fab.robots import Duration


def test_seconds():
    d = Duration(2, 5e8)
    assert d.seconds == 2.5


def test_ctor_takes_sec_as_float():
    d = Duration(2.6, 0)
    assert d.seconds == 2.6


def test_sec_remainder_add_to_nsec():
    d = Duration(2.6, 5e8)
    assert d.seconds == 3.1


def test_repr():
    d1 = Duration(2, 5e8)
    d2 = eval(repr(d1))
    assert d2.seconds == d1.seconds


def test_from_data():
    d = Duration.__from_data__(dict(secs=2, nsecs=5e8))
    assert d.seconds == 2.5


def test_to_data():
    d = Duration(2, 5e8)
    data = d.__data__

    assert data["secs"] == 2
    assert data["nsecs"] == 5e8


def test_equality():
    assert Duration(10, 25) == Duration(10, 25)


def test_inequality():
    assert Duration(10, 25) != Duration(10, 21)
