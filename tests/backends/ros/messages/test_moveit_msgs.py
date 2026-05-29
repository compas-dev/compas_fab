from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import MotionPlanRequest
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import WorkspaceParameters
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages import RosDistro
from compas_fab.backends.ros.messages import Time


def test_planning_scene_request_serializes_nested_headers_as_ros2_headers():
    header = Header(seq=10, stamp=Time(80, 20), frame_id="base_link")
    scene = PlanningScene(robot_state=RobotState(joint_state=JointState(header=header)))

    request = scene.to_request(RosDistro.JAZZY)

    assert request["scene"].msg["robot_state"]["joint_state"]["header"] == {
        "stamp": {"secs": 80, "nsecs": 20},
        "frame_id": "base_link",
    }


def test_moveit_error_codes_accepts_ros1_shape():
    error_code = MoveItErrorCodes.from_msg({"val": MoveItErrorCodes.PLANNING_FAILED})

    assert error_code == MoveItErrorCodes.PLANNING_FAILED
    assert error_code.message == ""
    assert error_code.source == ""


def test_moveit_error_codes_accepts_ros2_message_and_source():
    error_code = MoveItErrorCodes.from_msg(
        {
            "val": MoveItErrorCodes.PLANNING_FAILED,
            "message": "No valid plan found",
            "source": "move_group",
        }
    )

    assert error_code == MoveItErrorCodes.PLANNING_FAILED
    assert error_code.message == "No valid plan found"
    assert error_code.source == "move_group"


def test_cartesian_path_request_serializes_header_as_ros2_header():
    request = GetCartesianPathRequest(header=Header(seq=10, stamp=Time(80, 20), frame_id="base_link"))

    request.filter_fields_for_distro(RosDistro.JAZZY)

    assert request.msg["header"] == {
        "stamp": {"secs": 80, "nsecs": 20},
        "frame_id": "base_link",
    }


def test_motion_plan_request_serializes_workspace_header_as_ros2_header():
    request = MotionPlanRequest(workspace_parameters=WorkspaceParameters(header=Header(seq=10, stamp=Time(80, 20), frame_id="base_link")))

    request.filter_fields_for_distro(RosDistro.JAZZY)

    assert request.msg["motion_plan_request"]["workspace_parameters"]["header"] == {
        "stamp": {"secs": 80, "nsecs": 20},
        "frame_id": "base_link",
    }
