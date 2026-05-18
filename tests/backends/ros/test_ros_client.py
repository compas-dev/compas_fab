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


def test_ros2_robot_cell_loader_uses_http_file_server(monkeypatch):
    calls = []

    class FakeHttpFileServerLoader:
        def __init__(self, base_url, ros=None, local_cache=False, local_cache_directory=None):
            calls.append(("http", base_url, ros, local_cache, local_cache_directory))

    class FakeRosFileServerLoader:
        def __init__(self, ros=None, local_cache=False, local_cache_directory=None):
            calls.append(("ros", ros, local_cache, local_cache_directory))

    monkeypatch.setattr(client_module, "HttpFileServerLoader", FakeHttpFileServerLoader)
    monkeypatch.setattr(client_module, "RosFileServerLoader", FakeRosFileServerLoader)

    client = RosClient(host="example.com")
    client._ros_distro = RosDistro.JAZZY

    loader = client._get_robot_cell_loader(
        use_local_cache=True,
        local_cache_directory="/tmp/cache",
        http_file_server_base_url="http://assets.example.com:9091",
    )

    assert isinstance(loader, FakeHttpFileServerLoader)
    assert calls == [("http", "http://assets.example.com:9091", client, True, "/tmp/cache")]


def test_ros2_robot_cell_loader_defaults_http_file_server_to_rosbridge_host(monkeypatch):
    calls = []

    class FakeHttpFileServerLoader:
        def __init__(self, base_url, ros=None, local_cache=False, local_cache_directory=None):
            calls.append((base_url, ros, local_cache, local_cache_directory))

    monkeypatch.setattr(client_module, "HttpFileServerLoader", FakeHttpFileServerLoader)

    client = RosClient(host="ros-host.local")
    client._ros_distro = RosDistro.HUMBLE

    client._get_robot_cell_loader()

    assert calls == [("http://ros-host.local:9091", client, False, None)]


def test_ros1_robot_cell_loader_uses_ros_file_server(monkeypatch):
    calls = []

    class FakeHttpFileServerLoader:
        def __init__(self, base_url, ros=None, local_cache=False, local_cache_directory=None):
            calls.append(("http", base_url, ros, local_cache, local_cache_directory))

    class FakeRosFileServerLoader:
        def __init__(self, ros=None, local_cache=False, local_cache_directory=None):
            calls.append(("ros", ros, local_cache, local_cache_directory))

    monkeypatch.setattr(client_module, "HttpFileServerLoader", FakeHttpFileServerLoader)
    monkeypatch.setattr(client_module, "RosFileServerLoader", FakeRosFileServerLoader)

    client = RosClient(host="example.com")
    client._ros_distro = RosDistro.NOETIC

    loader = client._get_robot_cell_loader(use_local_cache=True, local_cache_directory="/tmp/cache")

    assert isinstance(loader, FakeRosFileServerLoader)
    assert calls == [("ros", client, True, "/tmp/cache")]


def _trajectory_with_single_point():
    from compas_fab.backends.ros.messages import Header
    from compas_fab.backends.ros.messages import JointTrajectory
    from compas_fab.backends.ros.messages import JointTrajectoryPoint
    from compas_fab.backends.ros.messages import Time

    point = JointTrajectoryPoint(positions=[0.0, 0.0], velocities=[0.0, 0.0], time_from_start=Time(1, 0))
    return JointTrajectory(Header(), ["a", "b"], [point])


def test_follow_joint_trajectory_uses_ros2_action_for_ros2_distro(monkeypatch):
    sent = {}

    class FakeRos2ActionClient:
        def __init__(self, ros, name, action_type):
            sent["name"] = name
            sent["action_type"] = action_type

        def send_goal(self, goal, resultback, feedback, errback):
            sent["goal"] = goal
            sent["resultback"] = resultback
            sent["feedback"] = feedback
            sent["errback"] = errback
            return "goal-id-1"

        def cancel_goal(self, goal_id):
            sent["cancelled"] = goal_id

    class FakeRos1ActionClient:
        def __init__(self, *a, **k):
            raise AssertionError("ROS 1 ActionClient should not be used for ROS 2 distros")

    monkeypatch.setattr(client_module, "Ros2ActionClient", FakeRos2ActionClient)
    monkeypatch.setattr(client_module, "ActionClient", FakeRos1ActionClient)

    client = RosClient(host="example.com")
    client._ros_distro = RosDistro.JAZZY

    result = client.follow_joint_trajectory(_trajectory_with_single_point())

    assert sent["name"] == "/joint_trajectory_action"
    assert sent["action_type"] == "control_msgs/action/FollowJointTrajectory"
    assert "trajectory" in sent["goal"]
    assert result.goal.goal_id == "goal-id-1"


def test_execute_joint_trajectory_uses_ros2_action_for_ros2_distro(monkeypatch):
    sent = {}

    class FakeRos2ActionClient:
        def __init__(self, ros, name, action_type):
            sent["name"] = name
            sent["action_type"] = action_type

        def send_goal(self, goal, resultback, feedback, errback):
            sent["goal"] = goal
            return "exec-goal-id"

        def cancel_goal(self, goal_id):
            sent["cancelled"] = goal_id

    monkeypatch.setattr(client_module, "Ros2ActionClient", FakeRos2ActionClient)

    client = RosClient(host="example.com")
    client._ros_distro = RosDistro.HUMBLE

    result = client.execute_joint_trajectory(_trajectory_with_single_point())

    assert sent["name"] == "/execute_trajectory"
    assert sent["action_type"] == "moveit_msgs/action/ExecuteTrajectory"
    assert "trajectory" in sent["goal"]
    assert result.goal.goal_id == "exec-goal-id"


def test_ros2_goal_handle_marks_finished_on_result():
    from compas_fab.backends.ros.client import _Ros2GoalHandle

    received = []

    class FakeActionClient:
        def send_goal(self, goal, resultback, feedback, errback):
            self._resultback = resultback
            return "g-1"

        def cancel_goal(self, goal_id):
            received.append(("cancel", goal_id))

    fake = FakeActionClient()
    handle = _Ros2GoalHandle(fake)
    handle.send({"trajectory": None}, on_result=lambda r: received.append(("result", r)))

    assert handle.is_finished is False
    fake._resultback({"ok": True})
    assert handle.is_finished is True
    assert received == [("result", {"ok": True})]

    handle.cancel()
    assert received[-1] == ("cancel", "g-1")
