import pytest

pytest.importorskip("pyroki")

from compas.geometry import Frame
from compas.geometry import Quaternion
from compas.geometry import Transformation
from compas.geometry import Translation

from compas_fab.backends import PyRokiPlanner
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode


def orientation_error(frame_a, frame_b):
    quaternion_a = Quaternion.from_frame(frame_a)
    quaternion_b = Quaternion.from_frame(frame_b)
    dot = abs(quaternion_a.w * quaternion_b.w + quaternion_a.x * quaternion_b.x + quaternion_a.y * quaternion_b.y + quaternion_a.z * quaternion_b.z)
    return 1.0 - min(1.0, dot)


def test_frame_target_inverse_kinematics_round_trip():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.3, -0.7, 0.8, -0.5, 0.6, -0.2]
    target_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(
        FrameTarget(target_frame, TargetMode.ROBOT),
        state,
        options={"max_iterations": 100, "check_collision": False},
    )

    actual = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    assert actual.point.distance_to_point(target_frame.point) <= 1e-4
    assert orientation_error(actual, target_frame) <= 1e-6
    assert result.joint_names == cell.get_configurable_joint_names()


def test_frame_target_is_converted_from_world_to_robot_base():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [-0.2, -0.5, 0.6, -0.4, 0.5, 0.1]
    target_base = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())
    state.robot_base_frame.point = [1.0, -0.5, 0.25]
    target_world = target_base.transformed(Transformation.from_frame(state.robot_base_frame))

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = next(planner.iter_inverse_kinematics(FrameTarget(target_world, TargetMode.ROBOT), state, options={"check_collision": False}))

    actual_base = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    assert actual_base.point.distance_to_point(target_base.point) <= 1e-4
    assert orientation_error(actual_base, target_base) <= 1e-6


def test_frame_target_respects_tool_mode():
    cell, state = RobotCellLibrary.ur5_cone_tool(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.2, -0.6, 0.7, -0.4, 0.5, -0.1]
    target_pcf = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())
    target_tcf = cell.pcf_to_target_frames(state, target_pcf, TargetMode.TOOL, cell.main_group_name)

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(FrameTarget(target_tcf, TargetMode.TOOL), state, options={"check_collision": False})

    actual_pcf = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    assert actual_pcf.point.distance_to_point(target_pcf.point) <= 1e-4
    assert orientation_error(actual_pcf, target_pcf) <= 1e-6


def test_frame_target_respects_workpiece_mode():
    cell, state = RobotCellLibrary.ur10e_gripper_one_beam(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.2, -0.8, 0.9, -0.5, 0.4, -0.1]
    target_pcf = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())
    target_ocf = cell.pcf_to_target_frames(state, target_pcf, TargetMode.WORKPIECE, cell.main_group_name)

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(FrameTarget(target_ocf, TargetMode.WORKPIECE), state, options={"check_collision": False})

    actual_pcf = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    assert actual_pcf.point.distance_to_point(target_pcf.point) <= 1e-4
    assert orientation_error(actual_pcf, target_pcf) <= 1e-6


def test_planning_group_keeps_inactive_joints_fixed():
    cell, state = RobotCellLibrary.panda(load_geometry=False)
    group = "panda_arm"
    state.robot_configuration["panda_finger_joint1"] = 0.017
    target_configuration = state.robot_configuration.copy()
    for name, value in zip(cell.get_configurable_joint_names(group), [0.05, -0.1, 0.05, -1.5, 0.05, 0.1, -0.05]):
        target_configuration[name] = value
    target_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name(group))

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(
        FrameTarget(target_frame, TargetMode.ROBOT),
        state,
        group=group,
        options={"check_collision": False, "return_full_configuration": True},
    )

    assert result["panda_finger_joint1"] == pytest.approx(0.017)
    actual = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name(group))
    assert actual.point.distance_to_point(target_frame.point) <= 1e-4
    assert orientation_error(actual, target_frame) <= 1e-6


def test_external_axis_group_keeps_other_robots_fixed():
    cell, state = RobotCellLibrary.rfl(load_geometry=False)
    group = "robot11_eaXYZ"
    active_names = cell.get_configurable_joint_names(group)
    target_configuration = state.robot_configuration.copy()
    for name, value in zip(active_names, [0.15, 0.05, 0.1, 0.2, -0.4, 0.5, -0.3, 0.4, -0.2]):
        target_configuration[name] = value
    target_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name(group))

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(
        FrameTarget(target_frame, TargetMode.ROBOT),
        state,
        group=group,
        options={"check_collision": False, "max_iterations": 150, "return_full_configuration": True},
    )

    inactive_names = set(cell.robot_model.get_configurable_joint_names()) - set(active_names)
    assert all(result[name] == pytest.approx(state.robot_configuration[name]) for name in inactive_names)
    actual = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name(group))
    assert actual.point.distance_to_point(target_frame.point) <= 1e-4
    assert orientation_error(actual, target_frame) <= 1e-6


def test_unreachable_frame_target_raises_inverse_kinematics_error():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    with pytest.raises(InverseKinematicsError) as error:
        planner.inverse_kinematics(
            FrameTarget(Frame([10.0, 10.0, 10.0]), TargetMode.ROBOT),
            state,
            options={"check_collision": False, "max_iterations": 20},
        )

    assert "did not reach" in error.value.message


def test_point_axis_target_leaves_rotation_about_axis_free():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.4, -0.8, 0.9, -0.3, 0.7, 0.5]
    target_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())
    target = PointAxisTarget(target_frame.point, target_frame.zaxis, TargetMode.ROBOT)

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(target, state, options={"check_collision": False})

    actual = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    assert actual.point.distance_to_point(target.target_point) <= 1e-4
    assert actual.zaxis.angle(target.target_z_axis) <= 1e-3


def test_point_axis_target_respects_tool_mode():
    cell, state = RobotCellLibrary.ur5_cone_tool(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.2, -0.6, 0.7, -0.4, 0.5, -0.1]
    target_pcf = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())
    target_tcf = cell.pcf_to_target_frames(state, target_pcf, TargetMode.TOOL, cell.main_group_name)
    target = PointAxisTarget(target_tcf.point, target_tcf.zaxis, TargetMode.TOOL)

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    result = planner.inverse_kinematics(target, state, options={"check_collision": False})

    actual_pcf = cell.robot_model.forward_kinematics(result, cell.get_end_effector_link_name())
    actual_tcf = cell.pcf_to_target_frames(state, actual_pcf, TargetMode.TOOL, cell.main_group_name)
    assert actual_tcf.point.distance_to_point(target.target_point) <= 1e-4
    assert actual_tcf.zaxis.angle(target.target_z_axis) <= 1e-3


def test_repeated_frame_targets_reuse_analyzed_problem():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.2, -0.6, 0.7, -0.4, 0.5, -0.1]
    first_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    first_result = planner.inverse_kinematics(FrameTarget(first_frame, TargetMode.ROBOT), state, options={"check_collision": False})
    state.robot_configuration = state.robot_configuration.merged(first_result)
    problem_template = next(iter(planner.client._problem_cache.ik_problems.values()))

    second_frame = first_frame.transformed(Translation.from_vector([0.002, -0.001, 0.001]))
    second_result = planner.inverse_kinematics(FrameTarget(second_frame, TargetMode.ROBOT), state, options={"check_collision": False})

    assert len(planner.client._problem_cache.ik_problems) == 1
    assert next(iter(planner.client._problem_cache.ik_problems.values())) is problem_template
    actual = cell.robot_model.forward_kinematics(state.robot_configuration.merged(second_result), cell.get_end_effector_link_name())
    assert actual.point.distance_to_point(second_frame.point) <= 1e-4


def test_repeated_point_axis_targets_reuse_analyzed_problem():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_configuration = state.robot_configuration.copy()
    target_configuration.joint_values = [0.3, -0.7, 0.8, -0.5, 0.6, -0.2]
    first_frame = cell.robot_model.forward_kinematics(target_configuration, cell.get_end_effector_link_name())

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    first_target = PointAxisTarget(first_frame.point, first_frame.zaxis, TargetMode.ROBOT)
    first_result = planner.inverse_kinematics(first_target, state, options={"check_collision": False})
    state.robot_configuration = state.robot_configuration.merged(first_result)
    problem_template = next(iter(planner.client._problem_cache.ik_problems.values()))

    second_point = first_frame.point.transformed(Translation.from_vector([0.001, 0.002, -0.001]))
    second_target = PointAxisTarget(second_point, first_frame.zaxis, TargetMode.ROBOT)
    second_result = planner.inverse_kinematics(second_target, state, options={"check_collision": False})

    assert len(planner.client._problem_cache.ik_problems) == 1
    assert next(iter(planner.client._problem_cache.ik_problems.values())) is problem_template
    actual = cell.robot_model.forward_kinematics(state.robot_configuration.merged(second_result), cell.get_end_effector_link_name())
    assert actual.point.distance_to_point(second_point) <= 1e-4
    assert actual.zaxis.angle(first_frame.zaxis) <= 1e-3


def test_setting_robot_cell_clears_analyzed_problem_cache():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_frame = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    planner.inverse_kinematics(FrameTarget(target_frame, TargetMode.ROBOT), state, options={"check_collision": False})
    assert planner.client._problem_cache.ik_problems

    planner.set_robot_cell(cell)

    assert not planner.client._problem_cache.ik_problems
    assert not planner.client._problem_cache.collision_scenes
