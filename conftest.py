def pytest_configure(config):
    from twisted.internet import selectreactor
    selectreactor.install()


def pytest_ignore_collect(path):
    if "rhino" in str(path):
        return True

    if "blender" in str(path):
        return True

    if "ghpython" in str(path):
        return True
