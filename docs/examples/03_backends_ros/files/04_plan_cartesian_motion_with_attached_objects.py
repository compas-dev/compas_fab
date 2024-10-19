from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Box

import compas_fab
from math import pi
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody
from compas_fab.robots import ToolLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import TargetMode
from compas_fab.robots import RigidBodyLibrary

target_frames = []
target_frames.append(Frame([0.4, 0.3, 0.45], [0, 1, 0], [-1, 0, 0]))
target_frames.append(Frame([0.4, 0.0, 0.45], [0, 1, 0], [-1, 0, 0]))
target_frames.append(Frame([0.4, -0.4, 0.45], [1, 0, 0], [0, 1, 0]))
target_frames.append(Frame([0.0, -0.4, 0.45], [1, 0, 0], [0, 1, 0]))

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)

    # =====================================
    # Step 1: Creation of Robot Cell
    # =====================================

    # Add a rigid body for the floor
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)
    # Add a gripper tool
    gripper = ToolLibrary.kinematic_gripper()
    robot_cell.tool_models[gripper.name] = gripper
    # Add a long bar material as workpiece
    bar_mesh = Box(0.05, 0.05, 0.6).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["bar"] = RigidBody.from_mesh(bar_mesh)
    # Add a target marker
    target = RigidBodyLibrary.target_marker(0.2)
    # MoveIt RViz GUI typically does not show objects without collision meshes, so
    # we set the collision meshes to be the same as the visual meshes
    target.collision_meshes = target.visual_meshes
    for i, frame in enumerate(target_frames):
        robot_cell.rigid_body_models["target_%d" % i] = target.copy()

    # =====================================
    # Step 2: Specify the Robot Cell State
    # =====================================

    # Attach the gripper to the robot
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)
    robot_cell_state.set_tool_attached_to_group(
        gripper.name,
        robot_cell.main_group_name,
        attachment_frame=Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        touch_links=["wrist_3_link"],  # This is the link that the tool is attached to
    )
    # Attach the workpiece to the tool
    robot_cell_state.set_rigid_body_attached_to_tool(
        "bar",
        gripper.name,
        attachment_frame=Frame([-0.025, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    )
    # Specify the base of the robot is allowed to collide with the floor
    robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link_inertia"]
    # Place target marker at location
    for i, frame in enumerate(target_frames):
        robot_cell_state.rigid_body_states["target_%d" % i].frame = frame
    # Initial configuration of the robot
    robot_cell_state.robot_configuration.joint_values = [0, pi / -2, pi / 2, 0, 0, 0]
    result = planner.set_robot_cell(robot_cell, robot_cell_state)

    fk_frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    print(fk_frame)
    input("Press Enter to continue...")

    # =====================================
    # Step 3: Plan Cartesian Motion
    # =====================================

    # frames.append(Frame([0.6, -0.2, 0.45], [1, 0, 0], [0, 1, 0]))
    waypoints = FrameWaypoints(target_frames, TargetMode.WORKPIECE)
    # Avoid collision is set to False to avoid colliding with the target markers
    options = {
        "max_step": 0.01,
        "avoid_collisions": False,
    }

    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)
    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
