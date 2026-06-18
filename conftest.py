def pytest_configure(config):
    from twisted.internet import selectreactor

    selectreactor.install()


def pytest_ignore_collect(collection_path):
    if "rhino" in str(collection_path):
        return True

    if "blender" in str(collection_path):
        return True

    if "ghpython" in str(collection_path):
        return True
