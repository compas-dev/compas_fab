#!/usr/bin/env python3
"""Generate headless glTF snapshots of solved robot-cell states.

By default this creates four snapshots of the built-in UR10e cell: analytical
and PyBullet IK, each with equivalent cones authored along positive Z and
positive X. Their matching base frames normalize both source orientations to
the same flange-relative tool. The cone includes a one-sided perpendicular
marker that makes this normalization visible.

Examples
--------
Generate the four local snapshots as single-file binary glTF artifacts::

    python scripts/generate_robot_cell_gltf_snapshots.py

Generate embedded JSON glTF files instead::

    python scripts/generate_robot_cell_gltf_snapshots.py --format gltf

Generate every requested local RobotCellLibrary scenario::

    python scripts/generate_robot_cell_gltf_snapshots.py --cells all

Select only cells that already contain an end effector::

    python scripts/generate_robot_cell_gltf_snapshots.py \
        --cells built-in-end-effectors --solvers pybullet

Load the robot description and geometry from ROS and solve with MoveIt::

    python scripts/generate_robot_cell_gltf_snapshots.py \
        --solvers ros --ros-host localhost --ros-port 9090

The ROS instance is expected to expose a robot description, semantic
description, geometry, MoveIt planning scene, and IK service.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from typing import Optional

import compas
import compas_robots
from compas.colors import Color
from compas.datastructures import Mesh
from compas.files import GLTF
from compas.files import GLTFContent
from compas.geometry import Cone
from compas.geometry import Cylinder
from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Rotation
from compas.geometry import Sphere
from compas.geometry import Transformation
from compas.geometry import angle_vectors
from compas.geometry import distance_point_point
from compas_robots import Configuration
from compas_robots import ToolModel
from compas_robots.model import Joint
from compas_robots.model import LinkGeometry

import compas_fab
from compas_fab.backends import ABB_IRB4600_40_255Kinematics
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends import MoveItPlanner
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.backends import RosClient
from compas_fab.backends import Staubli_TX2_60LKinematics
from compas_fab.backends import UR3eKinematics
from compas_fab.backends import UR3Kinematics
from compas_fab.backends import UR5eKinematics
from compas_fab.backends import UR5Kinematics
from compas_fab.backends import UR10eKinematics
from compas_fab.backends import UR10Kinematics
from compas_fab.backends import UR16eKinematics
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode

TOOL_ID = "snapshot_cone"
TOOL_LENGTH = 0.15
TOOL_RADIUS = 0.03
MARKER_LENGTH = 0.06
MARKER_RADIUS = 0.006
TARGET_AXIS_LENGTH = 0.08
TARGET_AXIS_RADIUS = 0.003
TARGET_ORIGIN_RADIUS = 0.007
ROBOT_COLOR = Color(0.72, 0.76, 0.82)
TOOL_COLOR = Color(1.0, 0.36, 0.05)
MARKER_COLOR = Color(0.0, 0.72, 0.82)
RIGID_BODY_COLOR = Color(0.55, 0.58, 0.62)
TARGET_ORIGIN_COLOR = Color(1.0, 0.85, 0.0)
TARGET_AXIS_COLORS = (Color(1.0, 0.1, 0.1), Color(0.1, 0.8, 0.1), Color(0.1, 0.3, 1.0))
REFERENCE_DELTAS = (0.35, -0.65, 0.8, -0.55, 0.45, 0.25, -0.3)


@dataclass(frozen=True)
class LocalCellSpec:
    """Describe how a RobotCellLibrary scenario supplies its end effector."""

    end_effector: str
    analytical_solver: object = None


LOCAL_CELL_SPECS = {
    "ur3": LocalCellSpec("generated_cone", UR3Kinematics),
    "ur3e": LocalCellSpec("generated_cone", UR3eKinematics),
    "ur5": LocalCellSpec("generated_cone", UR5Kinematics),
    "ur5e": LocalCellSpec("generated_cone", UR5eKinematics),
    "ur10": LocalCellSpec("generated_cone", UR10Kinematics),
    "ur10e": LocalCellSpec("generated_cone", UR10eKinematics),
    "ur16e": LocalCellSpec("generated_cone", UR16eKinematics),
    "staubli_tx2_60l": LocalCellSpec("generated_cone", Staubli_TX2_60LKinematics),
    "ur5_cone_tool": LocalCellSpec("built_in_tool", UR5Kinematics),
    "ur5_gripper_one_beam": LocalCellSpec("built_in_tool", UR5Kinematics),
    "ur10e_gripper_one_beam": LocalCellSpec("built_in_tool", UR10eKinematics),
    "abb_irb4600_40_255": LocalCellSpec("generated_cone", ABB_IRB4600_40_255Kinematics),
    "abb_irb4600_40_255_gripper_one_beam": LocalCellSpec("built_in_tool", ABB_IRB4600_40_255Kinematics),
    "abb_irb4600_40_255_printing_tool": LocalCellSpec("built_in_tool", ABB_IRB4600_40_255Kinematics),
    "panda": LocalCellSpec("robot"),
}
GENERATED_CONE_CELLS = tuple(name for name, spec in LOCAL_CELL_SPECS.items() if spec.end_effector == "generated_cone")
BUILT_IN_END_EFFECTOR_CELLS = tuple(name for name, spec in LOCAL_CELL_SPECS.items() if spec.end_effector != "generated_cone")
CELL_PRESETS = {
    "all": tuple(LOCAL_CELL_SPECS),
    "generated-cones": GENERATED_CONE_CELLS,
    "built-in-end-effectors": BUILT_IN_END_EFFECTOR_CELLS,
}


def create_cone_tool(axis: str) -> ToolModel:
    """Create equivalent tools authored along +Z or +X before base reframing."""
    if axis == "z":
        authored_frame = Frame.worldXY()
        authored_tcf = Frame([0.0, 0.0, TOOL_LENGTH], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        marker_frame = Frame([MARKER_LENGTH / 2.0, 0.0, TOOL_LENGTH * 0.75], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
        base_frame = Frame.worldXY()
    elif axis == "x":
        authored_frame = Frame.worldYZ()
        authored_tcf = Frame([TOOL_LENGTH, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
        # The +X-authored equivalent of the +Z tool's one-sided +X marker is
        # +Y in source coordinates. Both normalize to tool-base +X.
        marker_frame = Frame([TOOL_LENGTH * 0.75, MARKER_LENGTH / 2.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        base_frame = Frame.worldYZ()
    else:
        raise ValueError("Cone axis must be 'x' or 'z'.")

    cone_mesh = Mesh.from_shape(Cone(TOOL_RADIUS, TOOL_LENGTH, authored_frame), u=24)
    marker_mesh = Mesh.from_shape(Cylinder(MARKER_RADIUS, MARKER_LENGTH, marker_frame), u=16)
    cone_mesh.vertices_attribute("vertex_color", list(TOOL_COLOR.rgba))
    marker_mesh.vertices_attribute("vertex_color", list(MARKER_COLOR.rgba))
    cone_mesh.join(marker_mesh)

    return ToolModel(cone_mesh, authored_tcf, name="cone_{}".format(axis), base_frame=base_frame)


def add_and_attach_tool(robot_cell: RobotCell, axis: str) -> RobotCellState:
    """Add the requested cone and return a complete state with it attached."""
    robot_cell.tool_models[TOOL_ID] = create_cone_tool(axis)
    state = robot_cell.default_cell_state()
    state.set_tool_attached_to_group(TOOL_ID, robot_cell.main_group_name)
    return state


def target_mode_for_spec(spec: LocalCellSpec) -> TargetMode:
    return TargetMode.ROBOT if spec.end_effector == "robot" else TargetMode.TOOL


def reachable_target_frame(robot_cell: RobotCell, state: RobotCellState, target_mode: TargetMode) -> Frame:
    """Create a deterministic, reachable target from a non-singular reference state."""
    reference_state = state.copy()
    reference_configuration = reference_state.robot_configuration
    joints = robot_cell.get_configurable_joints(robot_cell.main_group_name)

    for index, joint in enumerate(joints):
        value = float(reference_configuration[joint.name]) + REFERENCE_DELTAS[index % len(REFERENCE_DELTAS)]
        if joint.limit and joint.type != Joint.CONTINUOUS:
            span = float(joint.limit.upper) - float(joint.limit.lower)
            margin = min(0.05, max(1e-6, span * 0.01))
            value = max(float(joint.limit.lower) + margin, min(float(joint.limit.upper) - margin, value))
        reference_configuration[joint.name] = value

    return robot_cell.forward_kinematics_target_frame(reference_state, target_mode)


def cyclic_distance_from_start(configuration: Configuration, start: Configuration) -> float:
    """Score a revolute configuration by its shortest angular distance from start."""
    start_by_name = dict(zip(start.joint_names, start.joint_values))
    score = 0.0
    for name, value in zip(configuration.joint_names, configuration.joint_values):
        delta = value - start_by_name.get(name, 0.0)
        shortest_delta = math.atan2(math.sin(delta), math.cos(delta))
        score += shortest_delta * shortest_delta
    return score


def solve_analytical(robot_cell: RobotCell, state: RobotCellState, target: FrameTarget, solver_class) -> RobotCellState:
    """Solve with the cell's analytical IK implementation and update the state."""
    planner = AnalyticalKinematicsPlanner(solver_class())
    planner.set_robot_cell(robot_cell)
    solutions = list(planner.iter_inverse_kinematics(target, state))
    if not solutions:
        raise RuntimeError("Analytical IK returned no solution.")
    state.robot_configuration = min(solutions, key=lambda solution: cyclic_distance_from_start(solution, state.robot_configuration))
    planner.set_robot_cell_state(state)
    return state


def solve_pybullet(robot_cell: RobotCell, state: RobotCellState, target: FrameTarget) -> tuple[RobotCellState, Frame]:
    """Solve with PyBullet IK and return both the state and backend FK frame."""
    with PyBulletClient(connection_type="direct") as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell, state)
        state.robot_configuration = planner.inverse_kinematics(
            target,
            state,
            options={
                "check_collision": False,
                "max_descend_iterations": 200,
                "max_results": 10,
            },
        )
        planner.set_robot_cell_state(state)
        backend_frame = planner.forward_kinematics(state, target.target_mode)
    return state, backend_frame


def solve_moveit(planner: MoveItPlanner, robot_cell: RobotCell, state: RobotCellState, target: FrameTarget) -> RobotCellState:
    """Synchronize the ROS planning scene, solve with MoveIt, and update it."""
    planner.set_robot_cell(robot_cell, state)
    state.robot_configuration = planner.inverse_kinematics(
        target,
        state,
        options={"check_collision": False, "max_results": 1},
    )
    planner.set_robot_cell_state(state)
    return state


def frame_data(frame: Frame) -> dict:
    """Return a JSON-safe representation of a frame."""
    return {
        "point": list(frame.point),
        "xaxis": list(frame.xaxis),
        "yaxis": list(frame.yaxis),
        "zaxis": list(frame.zaxis),
    }


def validate_solution(
    robot_cell: RobotCell,
    state: RobotCellState,
    target_frame: Frame,
    target_mode: TargetMode,
    backend_frame: Optional[Frame] = None,
    tolerance: float = 2e-3,
) -> dict:
    """Validate the solved end-effector frame and return error metrics."""
    model_frame = robot_cell.forward_kinematics_target_frame(state, target_mode)
    solver_frame = backend_frame or model_frame
    solver_position_error = distance_point_point(solver_frame.point, target_frame.point)
    solver_orientation_error = max(
        angle_vectors(solver_frame.xaxis, target_frame.xaxis),
        angle_vectors(solver_frame.yaxis, target_frame.yaxis),
        angle_vectors(solver_frame.zaxis, target_frame.zaxis),
    )
    if solver_position_error > tolerance or solver_orientation_error > tolerance:
        raise RuntimeError("Solver FK misses target: position error={:.6g} m, orientation error={:.6g} rad".format(solver_position_error, solver_orientation_error))

    model_position_error = distance_point_point(model_frame.point, target_frame.point)
    model_orientation_error = max(
        angle_vectors(model_frame.xaxis, target_frame.xaxis),
        angle_vectors(model_frame.yaxis, target_frame.yaxis),
        angle_vectors(model_frame.zaxis, target_frame.zaxis),
    )
    return {
        "solver_frame": frame_data(solver_frame),
        "solver_position_error_m": solver_position_error,
        "solver_orientation_error_rad": solver_orientation_error,
        "model_frame": frame_data(model_frame),
        "model_position_error_m": model_position_error,
        "model_orientation_error_rad": model_orientation_error,
        "backend_model_position_delta_m": distance_point_point(model_frame.point, solver_frame.point),
        "model_matches_target": model_position_error <= tolerance and model_orientation_error <= tolerance,
    }


def color_rgba(mesh: Mesh, item, fallback: Color) -> list[float]:
    """Resolve an imported mesh/URDF material color with a stable fallback."""
    diffuse = mesh.attributes.get("mesh_color.diffuse")
    if diffuse:
        rgba = list(diffuse)
    else:
        item_color = item.get_color() if hasattr(item, "get_color") else None
        rgba = list(item_color.rgba if item_color else fallback.rgba)
    if len(rgba) == 3:
        rgba.append(1.0)
    return rgba[:4]


def prepared_mesh(mesh: Mesh, transformation: Transformation, rgba: Iterable[float]) -> Mesh:
    """Bake a pose into a triangulated mesh and add glTF color and normal data."""
    posed = mesh.transformed(transformation)
    source_colors = posed.vertices_attribute("vertex_color")
    vertices, faces = posed.to_vertices_and_faces(triangulated=True)
    result = Mesh.from_vertices_and_faces(vertices, faces)
    fallback_color = list(rgba)
    for index, vertex in enumerate(result.vertices()):
        vertex_color = source_colors[index] if source_colors[index] is not None else fallback_color
        result.vertex_attribute(vertex, "vertex_color", list(vertex_color))
        if any(face is not None for face in result.vertex_faces(vertex)):
            normal = list(result.vertex_normal(vertex))
        else:
            # Some imported CAD meshes retain vertices that no face references.
            normal = [0.0, 0.0, 1.0]
        result.vertex_attribute(vertex, "vertex_normal", normal)
    return result


def link_transformations(model, configuration: Optional[Configuration]) -> dict[str, Transformation]:
    """Return each model link's transform relative to its model base."""
    configuration = configuration or model.zero_configuration()
    transformations = {model.root.name: Transformation()}
    frames = model.transformed_frames(configuration)
    for joint, frame in zip(model.iter_joints(), frames):
        transformations[joint.child_link.name] = Transformation.from_frame(frame)
    return transformations


def add_model_to_gltf(parent, model, configuration: Optional[Configuration], base_frame: Frame, fallback_color: Color) -> dict:
    """Add a posed RobotModel or ToolModel below a glTF node."""
    counts = {"links": 0, "meshes": 0, "vertices": 0, "faces": 0}
    world_from_model = Transformation.from_frame(base_frame)
    transforms = link_transformations(model, configuration)

    for link in model.iter_links():
        link_node = parent.add_child(link.name, {"kind": "link"})
        counts["links"] += 1
        world_from_link = world_from_model * transforms[link.name]
        for item_index, item in enumerate(link.visual):
            meshes = LinkGeometry._get_item_meshes(item) or []
            world_from_geometry = world_from_link
            if item.origin:
                world_from_geometry = world_from_geometry * Transformation.from_frame(item.origin)
            for mesh_index, mesh in enumerate(meshes):
                name = item.name or "visual_{}".format(item_index)
                mesh_name = "{}.{}.{}".format(link.name, name, mesh_index)
                mesh_node = link_node.add_child(mesh_name, {"kind": "visual_mesh"})
                gltf_mesh = prepared_mesh(mesh, world_from_geometry, color_rgba(mesh, item, fallback_color))
                mesh_data = mesh_node.add_mesh(gltf_mesh)
                mesh_data.mesh_name = mesh_name
                counts["meshes"] += 1
                counts["vertices"] += gltf_mesh.number_of_vertices()
                counts["faces"] += gltf_mesh.number_of_faces()
    return counts


def merge_counts(total: dict, addition: dict) -> None:
    for key, value in addition.items():
        total[key] = total.get(key, 0) + value


def add_target_frame_to_gltf(parent, target_frame: Frame) -> dict:
    """Add a yellow origin and RGB positive axes at the requested target frame."""
    counts = {"links": 0, "meshes": 0, "vertices": 0, "faces": 0}
    target_node = parent.add_child("target_frame", {"kind": "target_frame", "frame": frame_data(target_frame)})
    marker_meshes = [("origin", Mesh.from_shape(Sphere(TARGET_ORIGIN_RADIUS, point=target_frame.point), u=12, v=8), TARGET_ORIGIN_COLOR)]

    for name, axis, color in zip(("x_axis", "y_axis", "z_axis"), (target_frame.xaxis, target_frame.yaxis, target_frame.zaxis), TARGET_AXIS_COLORS):
        center = target_frame.point + axis * (TARGET_AXIS_LENGTH / 2.0)
        axis_frame = Frame.from_plane(Plane(center, axis))
        axis_mesh = Mesh.from_shape(Cylinder(TARGET_AXIS_RADIUS, TARGET_AXIS_LENGTH, axis_frame), u=12)
        marker_meshes.append((name, axis_mesh, color))

    for name, mesh, color in marker_meshes:
        mesh_node = target_node.add_child(name, {"kind": "target_marker"})
        gltf_mesh = prepared_mesh(mesh, Transformation(), color.rgba)
        mesh_data = mesh_node.add_mesh(gltf_mesh)
        mesh_data.mesh_name = "target_frame.{}".format(name)
        counts["meshes"] += 1
        counts["vertices"] += gltf_mesh.number_of_vertices()
        counts["faces"] += gltf_mesh.number_of_faces()
    return counts


def export_snapshot(filepath: Path, scenario: str, robot_cell: RobotCell, state: RobotCellState, target_frame: Frame, metadata: dict) -> dict:
    """Bake a RobotCellState into a colored, hierarchical glTF artifact."""
    state = robot_cell.compute_attach_objects_frames(state)
    content = GLTFContent()
    scene = content.add_scene(scenario, {"generator": "compas_fab robot-cell snapshot"})
    root = scene.add_child(scenario, metadata)

    # glTF viewers assume Y-up; COMPAS robot cells are Z-up.
    root.matrix = Rotation.from_axis_and_angle([1.0, 0.0, 0.0], -math.pi / 2.0).matrix

    counts = {"links": 0, "meshes": 0, "vertices": 0, "faces": 0}
    robot_node = root.add_child(robot_cell.robot_model.name, {"kind": "robot"})
    merge_counts(
        counts,
        add_model_to_gltf(robot_node, robot_cell.robot_model, state.robot_configuration, state.robot_base_frame, ROBOT_COLOR),
    )
    merge_counts(counts, add_target_frame_to_gltf(root, target_frame))

    tools_node = root.add_child("tools", {"kind": "tool_collection"})
    for tool_id, tool_model in robot_cell.tool_models.items():
        tool_state = state.tool_states[tool_id]
        if tool_state.is_hidden:
            continue
        tool_node = tools_node.add_child(tool_id, {"kind": "tool", "attached_to_group": tool_state.attached_to_group})
        merge_counts(counts, add_model_to_gltf(tool_node, tool_model, tool_state.configuration, tool_state.frame, TOOL_COLOR))

    bodies_node = root.add_child("rigid_bodies", {"kind": "rigid_body_collection"})
    for body_id, body in robot_cell.rigid_body_models.items():
        body_state = state.rigid_body_states[body_id]
        if body_state.is_hidden:
            continue
        body_node = bodies_node.add_child(body_id, {"kind": "rigid_body"})
        transformation = Transformation.from_frame(body_state.frame)
        for index, mesh in enumerate(body.visual_meshes_in_meters):
            mesh_name = "{}.visual.{}".format(body_id, index)
            mesh_node = body_node.add_child(mesh_name, {"kind": "visual_mesh"})
            gltf_mesh = prepared_mesh(mesh, transformation, RIGID_BODY_COLOR.rgba)
            mesh_data = mesh_node.add_mesh(gltf_mesh)
            mesh_data.mesh_name = mesh_name
            counts["meshes"] += 1
            counts["vertices"] += gltf_mesh.number_of_vertices()
            counts["faces"] += gltf_mesh.number_of_faces()

    filepath.parent.mkdir(parents=True, exist_ok=True)
    gltf = GLTF(str(filepath))
    gltf.content = content
    gltf.export(embed_data=True)

    # Parse the artifact again so a corrupt or unsupported export fails the run.
    exported = GLTF(str(filepath))
    exported.read()
    if len(exported.content.meshes) != counts["meshes"]:
        raise RuntimeError("glTF round-trip mesh count does not match the exported cell.")

    counts["nodes"] = len(exported.content.nodes)
    counts["bytes"] = filepath.stat().st_size
    return counts


def scenario_metadata(
    scenario: str,
    solver: str,
    cell_factory: str,
    axis: Optional[str],
    end_effector: str,
    target_mode: TargetMode,
    target_frame: Frame,
    robot_cell: RobotCell,
    state: RobotCellState,
    validation: dict,
) -> dict:
    configuration = state.robot_configuration
    return {
        "scenario": scenario,
        "solver": solver,
        "cell_factory": cell_factory,
        "end_effector": end_effector,
        "cone_axis": "+{}".format(axis.upper()) if axis else None,
        "authored_cone_axis": "+{}".format(axis.upper()) if axis else None,
        "mounted_cone_axis": "+Z" if axis else None,
        "robot": robot_cell.robot_model.name,
        "group": robot_cell.main_group_name,
        "target_mode": target_mode.name,
        "target_frame": frame_data(target_frame),
        "attached_tool": state.get_attached_tool_id(robot_cell.main_group_name),
        "joint_names": list(configuration.joint_names),
        "joint_values": list(configuration.joint_values),
        "validation": validation,
        "coordinate_system": {"source_up_axis": "+Z", "gltf_up_axis": "+Y"},
    }


def generate_local_scenario(cell_factory: str, solver: str, axis: Optional[str], output_dir: Path, extension: str) -> dict:
    spec = LOCAL_CELL_SPECS[cell_factory]
    robot_cell, state = getattr(RobotCellLibrary, cell_factory)(load_geometry=True)

    if spec.end_effector == "generated_cone":
        if not axis:
            raise ValueError("Generated-cone scenarios require an axis.")
        state = add_and_attach_tool(robot_cell, axis)
        scenario_suffix = "cone_{}".format(axis)
    elif spec.end_effector == "built_in_tool":
        tool_id = state.get_attached_tool_id(robot_cell.main_group_name)
        if not tool_id:
            raise RuntimeError("RobotCellLibrary.{} did not return its built-in tool attached.".format(cell_factory))
        scenario_suffix = "built_in_{}".format(tool_id)
    else:
        scenario_suffix = "robot_end_effector"

    target_mode = target_mode_for_spec(spec)
    target_frame = reachable_target_frame(robot_cell, state, target_mode)
    target = FrameTarget(target_frame, target_mode)
    backend_frame = None
    if solver == "analytical":
        state = solve_analytical(robot_cell, state, target, spec.analytical_solver)
    elif solver == "pybullet":
        state, backend_frame = solve_pybullet(robot_cell, state, target)
    else:
        raise ValueError("Unknown local solver: {}".format(solver))

    validation = validate_solution(robot_cell, state, target_frame, target_mode, backend_frame=backend_frame)
    scenario = "{}_{}_{}".format(cell_factory, solver, scenario_suffix)
    metadata = scenario_metadata(
        scenario,
        solver,
        cell_factory,
        axis,
        spec.end_effector,
        target_mode,
        target_frame,
        robot_cell,
        state,
        validation,
    )
    filepath = output_dir / "{}.{}".format(scenario, extension)
    counts = export_snapshot(filepath, scenario, robot_cell, state, target_frame, metadata)
    return {"artifact": filepath.name, "metadata": metadata, "geometry": counts}


def generate_ros_scenarios(args, output_dir: Path, extension: str) -> list[dict]:
    client_kwargs = {"host": args.ros_host, "port": args.ros_port, "is_secure": args.ros_secure}
    with RosClient(**client_kwargs) as client:
        robot_cell = client.load_robot_cell(
            load_geometry=True,
            local_cache_directory=args.ros_cache,
            http_file_server_base_url=args.ros_http_file_server,
        )
        planner = MoveItPlanner(client)
        results = []
        for axis in args.axes:
            state = add_and_attach_tool(robot_cell, axis)
            target_frame = reachable_target_frame(robot_cell, state, TargetMode.TOOL)
            target = FrameTarget(target_frame, TargetMode.TOOL)
            state = solve_moveit(planner, robot_cell, state, target)
            validation = validate_solution(robot_cell, state, target_frame, TargetMode.TOOL)
            safe_robot_name = robot_cell.robot_model.name.replace("/", "_").replace(" ", "_")
            scenario = "{}_ros_moveit_cone_{}".format(safe_robot_name, axis)
            metadata = scenario_metadata(
                scenario,
                "ros_moveit",
                "ros",
                axis,
                "generated_cone",
                TargetMode.TOOL,
                target_frame,
                robot_cell,
                state,
                validation,
            )
            filepath = output_dir / "{}.{}".format(scenario, extension)
            counts = export_snapshot(filepath, scenario, robot_cell, state, target_frame, metadata)
            results.append({"artifact": filepath.name, "metadata": metadata, "geometry": counts})
        return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--solvers",
        nargs="+",
        choices=("analytical", "pybullet", "ros"),
        default=("analytical", "pybullet"),
        help="IK backends to snapshot (default: analytical pybullet). 'ros' uses MoveIt.",
    )
    parser.add_argument(
        "--cells",
        nargs="+",
        choices=tuple(LOCAL_CELL_SPECS) + tuple(CELL_PRESETS),
        default=("ur10e",),
        help="Local RobotCellLibrary factories or presets (default: ur10e). Ignored by ROS scenarios.",
    )
    parser.add_argument("--axes", nargs="+", choices=("z", "x"), default=("z", "x"), help="Cone mounting axes (default: z x).")
    parser.add_argument("--format", choices=("glb", "gltf"), default="glb", help="Single-file glTF format (default: glb).")
    parser.add_argument("--output-dir", type=Path, default=Path("temp/robot_cell_snapshots"), help="Artifact directory.")
    parser.add_argument("--ros-host", default="localhost", help="rosbridge host (ROS scenarios only).")
    parser.add_argument("--ros-port", type=int, default=9090, help="rosbridge port (ROS scenarios only).")
    parser.add_argument("--ros-secure", action="store_true", help="Use a secure rosbridge WebSocket.")
    parser.add_argument("--ros-cache", help="Robot-specific local geometry cache directory.")
    parser.add_argument("--ros-http-file-server", help="ROS 2 HTTP file-server base URL used to resolve package meshes.")
    return parser.parse_args()


def expand_cells(values: Iterable[str]) -> list[str]:
    """Expand cell presets while retaining order and removing duplicates."""
    expanded = []
    for value in values:
        names = CELL_PRESETS.get(value, (value,))
        for name in names:
            if name not in expanded:
                expanded.append(name)
    return expanded


def print_result(output_dir: Path, result: dict) -> None:
    geometry = result["geometry"]
    print(
        "Generated {} ({} meshes, {} vertices, {:.1f} KiB)".format(
            output_dir / result["artifact"],
            geometry["meshes"],
            geometry["vertices"],
            geometry["bytes"] / 1024.0,
        )
    )
    validation = result["metadata"]["validation"]
    if not validation["model_matches_target"]:
        print("  Warning: RobotModel FK differs from solver FK by {:.3f} mm at the target.".format(validation["backend_model_position_delta_m"] * 1000.0))


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    results = []
    skipped = []
    local_cells = expand_cells(args.cells)

    for solver in args.solvers:
        if solver == "ros":
            solver_results = generate_ros_scenarios(args, output_dir, args.format)
            results.extend(solver_results)
            for result in solver_results:
                print_result(output_dir, result)
            continue
        for cell_factory in local_cells:
            spec = LOCAL_CELL_SPECS[cell_factory]
            if solver == "analytical" and not spec.analytical_solver:
                skip = {
                    "cell_factory": cell_factory,
                    "solver": solver,
                    "reason": "No analytical kinematics solver is available for this robot.",
                }
                skipped.append(skip)
                print("Skipped {} with {}: {}".format(cell_factory, solver, skip["reason"]))
                continue
            axes = args.axes if spec.end_effector == "generated_cone" else (None,)
            for axis in axes:
                result = generate_local_scenario(cell_factory, solver, axis, output_dir, args.format)
                results.append(result)
                print_result(output_dir, result)

    manifest = {
        "versions": {
            "compas": compas.__version__,
            "compas_fab": compas_fab.__version__,
            "compas_robots": compas_robots.__version__,
        },
        "results": results,
        "skipped": skipped,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print("Generated manifest {}".format(manifest_path))


if __name__ == "__main__":
    main()
