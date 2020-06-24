import compas_fab.backends


def pytest_configure(config):
    compas_fab.backends._called_from_test = True
