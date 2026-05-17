from compas_fab.backends.ros import client as client_module
from compas_fab.backends.ros.client import RosClient
from compas_fab.backends.ros.messages import RosDistro


def test_ros_distro_prefers_rosapi_service(monkeypatch):
    calls = []

    class FakeService:
        def __init__(self, ros, name, service_type):
            calls.append(("service", name, service_type))

        def call(self, request, timeout=None):
            calls.append(("service_call", timeout))
            return {"version": 2, "distro": "jazzy"}

    class FakeParam:
        def __init__(self, ros, name):
            calls.append(("param", name))

        def get(self, timeout=None):
            raise AssertionError("The /rosdistro parameter should not be queried when rosapi returns a distro")

    monkeypatch.setattr(client_module, "Service", FakeService)
    monkeypatch.setattr(client_module, "Param", FakeParam)

    assert RosClient().ros_distro == RosDistro.JAZZY
    assert calls == [
        ("service", "/rosapi/get_ros_version", "rosapi_msgs/GetROSVersion"),
        ("service_call", 1),
    ]


def test_ros_distro_falls_back_to_rosdistro_param(monkeypatch):
    calls = []

    class FakeService:
        def __init__(self, ros, name, service_type):
            calls.append(("service", name, service_type))

        def call(self, request, timeout=None):
            calls.append(("service_call", timeout))
            raise RuntimeError("service unavailable")

    class FakeParam:
        def __init__(self, ros, name):
            calls.append(("param", name))

        def get(self, timeout=None):
            calls.append(("param_get", timeout))
            return "noetic"

    monkeypatch.setattr(client_module, "Service", FakeService)
    monkeypatch.setattr(client_module, "Param", FakeParam)

    assert RosClient().ros_distro == RosDistro.NOETIC
    assert calls == [
        ("service", "/rosapi/get_ros_version", "rosapi_msgs/GetROSVersion"),
        ("service_call", 1),
        ("param", "/rosdistro"),
        ("param_get", 1),
    ]


def test_ros_distro_uses_jazzy_default_when_detection_fails(monkeypatch):
    class FakeService:
        def __init__(self, ros, name, service_type):
            pass

        def call(self, request, timeout=None):
            raise RuntimeError("service unavailable")

    class FakeParam:
        def __init__(self, ros, name):
            pass

        def get(self, timeout=None):
            raise RuntimeError("param unavailable")

    monkeypatch.setattr(client_module, "Service", FakeService)
    monkeypatch.setattr(client_module, "Param", FakeParam)

    assert RosClient().ros_distro == RosDistro.JAZZY
