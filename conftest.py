def pytest_configure(config):
    from twisted.internet import selectreactor
    selectreactor.install()
