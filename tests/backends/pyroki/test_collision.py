import pytest

pytest.importorskip("pyroki")

from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Translation

from compas_fab.backends import CollisionCheckError
from compas_fab.backends import PyRokiPlanner
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.pyroki.collision import compile_collision_scene
from compas_fab.backends.pyroki.problem_cache import collision_scene_key
from compas_fab.robots import FrameTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode


COLLISION_FREE_CONFIGURATION = [0.3, -0.7, 0.8, -0.5, 0.6, -0.2]


def _ur5_with_obstacle_at_wrist():
    cell, state = RobotCellLibrary.ur5(load_geometry=True)
    state.robot_configuration.joint_values = COLLISION_FREE_CONFIGURATION
    obstacle_frame = cell.robot_model.forward_kinematics(state.robot_configuration, "wrist_3_link")
    cell.rigid_body_models["obstacle"] = RigidBody.from_mesh(Mesh.from_shape(Box(0.16)))
    state.rigid_body_states["obstacle"] = RigidBodyState(obstacle_frame)
    return cell, state


def test_check_collision_accepts_clear_state_and_respects_disabled_pairs():
    cell, state = RobotCellLibrary.ur5(load_geometry=True)
    state.robot_configuration.joint_values = COLLISION_FREE_CONFIGURATION
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    planner.check_collision(state)

    scene = compile_collision_scene(planner.client.robot_cell, state)
    disabled = cell.robot_semantics.unordered_disabled_collisions
    assert scene.self_pairs
    assert all(frozenset(pair) not in disabled for pair in scene.self_pairs)


def test_check_collision_reports_actual_robot_self_collision():
    cell, state = RobotCellLibrary.ur5(load_geometry=True)
    state.robot_configuration.joint_values = [
        -0.3195542760713268,
        1.9788578299617292,
        1.0455878271324153,
        -4.491216422580997,
        -6.146708954273636,
        -1.5738814705154258,
    ]
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    with pytest.raises(CollisionCheckError) as error:
        planner.check_collision(state, options={"full_report": True})

    assert ("forearm_link", "wrist_2_link") in error.value.collision_pairs


def test_world_collision_and_touch_links_come_from_fab_state():
    cell, state = _ur5_with_obstacle_at_wrist()
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    with pytest.raises(CollisionCheckError) as error:
        planner.check_collision(state, options={"full_report": True})
    assert ("wrist_3_link", "obstacle") in error.value.collision_pairs

    state.rigid_body_states["obstacle"].touch_links.append("wrist_3_link")
    scene = compile_collision_scene(planner.client.robot_cell, state)
    assert ("wrist_3_link", "obstacle") not in scene.world_pairs


def test_inverse_kinematics_checks_collision_by_default_and_allows_opt_out():
    cell, state = _ur5_with_obstacle_at_wrist()
    target_frame = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    with pytest.raises((InverseKinematicsError, CollisionCheckError)):
        planner.inverse_kinematics(target, state)

    result = planner.inverse_kinematics(target, state, options={"check_collision": False})
    state.robot_configuration = result
    with pytest.raises(CollisionCheckError):
        planner.check_collision(state)


def test_redundant_robot_ik_moves_around_world_obstacle():
    cell, state = RobotCellLibrary.panda(load_geometry=True)
    state.robot_configuration.joint_values = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02]
    target_frame = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())
    obstacle_frame = cell.robot_model.forward_kinematics(state.robot_configuration, "panda_link4")
    cell.rigid_body_models["obstacle"] = RigidBody.from_mesh(Mesh.from_shape(Box(0.12)))
    state.rigid_body_states["obstacle"] = RigidBodyState(obstacle_frame)

    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    with pytest.raises(CollisionCheckError):
        planner.check_collision(state)

    result = planner.inverse_kinematics(
        FrameTarget(target_frame, TargetMode.ROBOT),
        state,
        options={"max_iterations": 200},
    )
    state.robot_configuration = state.robot_configuration.merged(result)
    planner.check_collision(state)

    actual = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())
    assert actual.point.distance_to_point(target_frame.point) <= 1e-4

    second_target_frame = target_frame.transformed(Translation.from_vector([0.001, 0.0, 0.0]))
    second_result = planner.inverse_kinematics(
        FrameTarget(second_target_frame, TargetMode.ROBOT),
        state,
        options={"max_iterations": 200},
    )
    state.robot_configuration = state.robot_configuration.merged(second_result)
    planner.check_collision(state)
    second_actual = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())

    assert second_actual.point.distance_to_point(second_target_frame.point) <= 1e-4
    assert len(planner.client._problem_cache.collision_scenes) == 1
    assert len(planner.client._problem_cache.ik_problems) == 1


def test_default_collision_check_requires_loaded_geometry():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    target_frame = cell.robot_model.forward_kinematics(state.robot_configuration, cell.get_end_effector_link_name())
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)

    with pytest.raises(ValueError, match="Collision geometry.*not loaded"):
        planner.inverse_kinematics(FrameTarget(target_frame, TargetMode.ROBOT), state)


def test_attached_tool_and_floor_are_compiled_with_touch_semantics():
    cell, state = RobotCellLibrary.ur5_cone_tool(load_geometry=True)
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    scene = compile_collision_scene(planner.client.robot_cell, state)

    assert any("cone" in pair for pair in scene.self_pairs)
    assert any("floor" in pair for pair in scene.world_pairs)
    assert ("wrist_3_link", "cone") not in scene.self_pairs


def test_detached_tool_collision_is_reported_as_static_scene_error():
    cell, state = RobotCellLibrary.ur5_cone_tool(load_geometry=True)
    state.set_tool_detached("cone", frame=Frame.worldXY())
    planner = PyRokiPlanner()
    planner.set_robot_cell(cell)
    scene = compile_collision_scene(planner.client.robot_cell, state)

    assert ("cone", "floor") in scene.static_colliding_pairs()
    with pytest.raises(CollisionCheckError, match="cone.*floor"):
        planner.check_collision(state, options={"full_report": True})


def test_collision_scene_cache_key_ignores_only_robot_configuration():
    cell, state = _ur5_with_obstacle_at_wrist()
    initial_key = collision_scene_key(state)
    assert collision_scene_key(state.copy()) == initial_key

    state.robot_configuration.joint_values[0] += 0.1
    assert collision_scene_key(state) == initial_key

    state.rigid_body_states["obstacle"].frame.point.x += 0.01
    assert collision_scene_key(state) != initial_key
