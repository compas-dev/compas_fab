import pytest

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the viewer (currently only pybullet backend supports this)')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")
