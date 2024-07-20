from compas.geometry import Frame
from compas.geometry import Box
from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import AnalyticalPyBulletClient
from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.backends.kinematics.solvers import UR5Kinematics
from compas_fab.robots import RigidBody
from compas_robots import ToolModel
from compas_robots import Configuration
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary
from compas_fab.robots import RobotCell
from compas_fab.robots import FrameTarget


with AnalyticalPyBulletClient(connection_type="gui") as client:
    robot = RobotLibrary.ur5(load_geometry=True)
    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())

    # ---------------------------------------------------------------------
    # Create a robot cell and add objects to it
    # ---------------------------------------------------------------------
    robot_cell = RobotCell(robot)

    # Add Static Collision Geometry
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody(floor_mesh)

    # Add Tool
    tool_mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    tool_frame = Frame([0, 0, 0.14], [1, 0, 0], [0, 1, 0])
    robot_cell.tool_models["cone"] = ToolModel(tool_mesh, tool_frame)

    # Add workpiece at tool tip
    workpiece_mesh = Box(0.5, 0.1, 0.2).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["workpiece"] = RigidBody(workpiece_mesh)

    # ------------------------------------------------------------------------
    # Create a RobotCellState to represent the current state of the robot cell
    # ------------------------------------------------------------------------
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

    # Attach the tool to the robot's main group
    robot_cell_state.set_tool_attached_to_group("cone", robot.main_group_name)

    # Attach the workpiece to the tool
    workpiece_grasp_frame = Frame([0, 0, 0.1], [1, 0, 0], [0, 1, 0])
    robot_cell_state.set_rigid_body_attached_to_tool("workpiece", "cone", workpiece_grasp_frame)

    # The planner is used for passing the robot cell into the PyBullet client
    planner.set_robot_cell(robot_cell)

    # Wait for user to close the GUI
    # input("Press Enter to close the GUI...")

    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    target = FrameTarget(frame_WCF)

    options = {"check_collision": True, "keep_order": False}
    for joint_values, joint_names in planner.iter_inverse_kinematics(target, robot_cell_state, options=options):
        config = Configuration.from_revolute_values(joint_values, joint_names)
        print(config)
        # Visualize configuration in GUI
        robot_cell_with_config = robot_cell_state.copy()  # type: RobotCellState
        robot_cell_with_config.robot_configuration = config
        planner.set_robot_cell_state(robot_cell_with_config)
        input("Press Enter to continue...")
