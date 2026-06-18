def trajectory_replay(planner, robot_cell_state, trajectory):
    """The function only helps with demonstration using PyBullet's GUI mode.
    User can step through the trajectory points by pressing 'Enter' key

    In a real application, the trajectory should be visualized in
    a frontend such as Rhino or compas_viewer.
    The use of PyBullet's GUI mode is discouraged in production code.

    """
    print("Replaying trajectory with {} points. Auto loop enabled.".format(len(trajectory.points)))
    step = 0
    intermediate_robot_cell_state = robot_cell_state.copy()  # type: RobotCellState
    while True:
        if step >= len(trajectory.points):
            step = 0

        print("Step: {} - joint_values = {}".format(step, trajectory.points[step].joint_values))
        intermediate_robot_cell_state.robot_configuration = trajectory.points[step]
        planner.set_robot_cell_state(intermediate_robot_cell_state)

        input("Press Enter to continue...")
        step += 1
