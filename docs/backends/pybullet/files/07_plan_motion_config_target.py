from _pybullet_demo_helper import trajectory_replay
from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import MPNoPlanFoundError
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell in this example is loaded from RobotCellLibrary
    # The printing tool TCP is defined with its Z axis pointing out of nozzle
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()

    # Add a box (without collision geometry) for demonstration visualization
    # Printing tool will trace a square around this box
    # box_size = 0.6 will demonstrate a successful planning
    # box_size = 0.8 will demonstrate a failure because it is out of reach
    box_size = 0.4
    box = Box.from_corner_corner_height([1.0, 0.0, 0], [1.0 + box_size, 0.0 + box_size, 0], 1.50)
    rigidbody = RigidBody(box.to_mesh(True), None)
    robot_cell.rigid_body_models["box"] = rigidbody
    robot_cell_state.rigid_body_states["box"] = RigidBodyState(Frame.worldXY())

    planner.set_robot_cell(robot_cell)

    # Set the robot's initial configuration 
    start_state = robot_cell_state.copy() # type: RobotCellState
    start_state.robot_configuration.joint_values = [-0.5, 0.5, 0.0, 1.0, -0.7, 0.0]
    end_state = robot_cell_state.copy() # type: RobotCellState
    end_state.robot_configuration.joint_values = [0.5, 0.5, 0.0, 1.0, -0.7, 0.0]

    planner.set_robot_cell_state(start_state)
    planner.check_collision(start_state)
    input("Observe the start state in PyBullet's GUI, Press Enter to continue...")

    planner.set_robot_cell_state(end_state)
    planner.check_collision(end_state)
    input("Observe the end state in PyBullet's GUI, Press Enter to continue...")


    # --------------------------------------
    # Plan Free Motion with ConfigurationTarget
    # ---------------------------------------------

    target = ConfigurationTarget(end_state.robot_configuration)
    try:
        trajectory = planner.plan_motion(target, start_state)
        print("Planned trajectory has {} points.".format(len(trajectory.points)))
    except MPNoIKSolutionError as e:
        # This exception is raised when part of the trajectory has no IK solution,
        # either due to collision or it is not reachable.
        print("No IK solution found. Reason:", e.message)
        print("Target that could not be reached:", e.target)
        trajectory = e.partial_trajectory
        print("Partial trajectory returned has {} points.".format(len(trajectory.points)))
    except MPNoPlanFoundError as e:
        # This exception is raised when no plan could be found, the IK solutions along the
        # trajectory are valid, but the planner could not find a continuous path between them.
        print("No plan found. Reason:", e.message)
        trajectory = e.partial_trajectory
        print("Partial trajectory returned has {} points.".format(len(trajectory.points)))

    # # ------------------------------------------------
    # # Replay the trajectory in the PyBullet simulation
    # # ------------------------------------------------
    
    trajectory_replay(planner, robot_cell_state, trajectory)
