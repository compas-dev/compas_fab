from compas_fab.robots import Duration


def test_seconds():
    d = Duration(2, 5e+8)
    assert d.seconds == 2.5


def test_ctor_only_takes_integers():
    d = Duration(2.5, 5e+8)
    assert d.seconds == 2.5


def test_repr():
    d1 = Duration(2, 5e+8)
    d2 = eval(repr(d1))
    assert d2.seconds == d1.seconds


def test_from_data():
    d = Duration.from_data(dict(secs=2, nsecs=5e+8))
    assert d.seconds == 2.5


def test_to_data():
    d = Duration(2, 5e+8)
    data = d.to_data()

    assert data['secs'] == 2
    assert data['nsecs'] == 5e+8


def test_equality():
    assert Duration(10, 25) == Duration(10, 25)


def test_inequality():
    assert Duration(10, 25) != Duration(10, 21)
