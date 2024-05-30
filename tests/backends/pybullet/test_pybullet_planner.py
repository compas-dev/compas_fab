from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Frame

from compas.tolerance import Tolerance
from compas_fab.robots import RobotLibrary

# The tolerance for the tests are set to 1e-4 meters, equivalent to 0.1 mm
TOL = Tolerance(unit="m", absolute=1e-4, relative=1e-4)


def validate_planner_model_fk_with_truth(planner_result, model_result, true_result):
    """Helper function to validate planner results with known truth and model results

    Planner result comes from the planning backend.
    Model result comes from the RobotModel FK calculation.
    True result is the known ground truth result.
    """
    # Check with known ground truth result
    assert TOL.is_allclose(
        planner_result.point, true_result.point
    ), f"Planner Result Pt {planner_result.point} != Known Truth Pt{true_result.point}"
    assert TOL.is_allclose(
        planner_result.xaxis, true_result.xaxis
    ), f"Planner Result X {planner_result.xaxis} != Known Truth X {true_result.xaxis}"
    assert TOL.is_allclose(
        planner_result.yaxis, true_result.yaxis
    ), f"Planner Result Y {planner_result.yaxis} != Known Truth Y {true_result.yaxis}"
    # Check with RobotModel FK result
    assert TOL.is_allclose(
        model_result.point, true_result.point
    ), f"Model Result Pt {model_result.point} != Known Truth Pt {true_result.point}"
    assert TOL.is_allclose(
        model_result.xaxis, true_result.xaxis
    ), f"Model Result X {model_result.xaxis} != Known Truth X {true_result.xaxis}"
    assert TOL.is_allclose(
        model_result.yaxis, true_result.yaxis
    ), f"Model Result Y {model_result.yaxis} != Known Truth Y {true_result.yaxis}"


def validate_ik_with_fk(ik_target_frame, fk_result_frame):
    # Check if the IK target frame is close to the FK result frame
    assert TOL.is_allclose(
        ik_target_frame.point, fk_result_frame.point
    ), f"IK Target Pt {ik_target_frame.point} != FK Result Pt {fk_result_frame.point}"
    assert TOL.is_allclose(
        ik_target_frame.xaxis, fk_result_frame.xaxis
    ), f"IK Target X {ik_target_frame.xaxis} != FK Result X {fk_result_frame.xaxis}"
    assert TOL.is_allclose(
        ik_target_frame.yaxis, fk_result_frame.yaxis
    ), f"IK Target Y {ik_target_frame.yaxis} != FK Result Y {fk_result_frame.yaxis}"


def _test_fk_with_pybullet_planner(robot, true_result):
    with PyBulletClient(connection_type="direct") as client:
        robot = client.load_existing_robot(robot)
        planner = PyBulletPlanner(client)

        planning_group = robot.main_group_name
        end_effector_link = robot.get_end_effector_link_name(planning_group)

        planner_fk_result = planner.forward_kinematics(robot, robot.zero_configuration(), planning_group)
        print(planner_fk_result)
        model_fk_result = robot.model.forward_kinematics(robot.zero_configuration(), end_effector_link)
        print(model_fk_result)

        validate_planner_model_fk_with_truth(planner_fk_result, model_fk_result, true_result)


def test_pybullet_planner_fk_ur5():
    robot = RobotLibrary.ur5(load_geometry=True)
    true_result = Frame(
        point=Point(x=0.8172500133514404, y=0.19144999980926514, z=-0.005491000134497881),
        xaxis=Vector(x=-1.0, y=0.0, z=0.0),
        yaxis=Vector(x=0.0, y=0.0, z=1.0),
    )
    _test_fk_with_pybullet_planner(robot, true_result)


def test_pybullet_planner_fk_abb_irb4600_40_255():
    robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)
    true_result = Frame(
        point=Point(x=1.58, y=0.0, z=1.765),
        xaxis=Vector(x=0.0, y=0.0, z=-1.0),
        yaxis=Vector(x=0.0, y=1.0, z=0.0),
    )
    _test_fk_with_pybullet_planner(robot, true_result)


def test_pybullet_planner_fk_ur10e():
    robot = RobotLibrary.ur10e(load_geometry=True)
    true_result = Frame(
        point=Point(x=1.18425, y=0.2907, z=0.0608),
        xaxis=Vector(x=-1.0, y=0.0, z=-0.0),
        yaxis=Vector(x=0.0, y=0.0, z=1.0),
    )
    _test_fk_with_pybullet_planner(robot, true_result)


######################################################
# Testing the IK-FK agreement for the PyBullet backend
######################################################


def _test_pybullet_ik_fk_agreement(robot, ik_target_frames):
    # These options are set to ensure that the IK solver converges to a high accuracy
    # Threshold is set to 1e-5 meters to be larger than the tolerance used for comparison (1e-4 meters)
    ik_options = {"high_accuracy_max_iter": 50, "high_accuracy": True, "high_accuracy_threshold": 1e-5}

    with PyBulletClient(connection_type="direct") as client:
        robot = client.load_existing_robot(robot)
        planner = PyBulletPlanner(client)
        planning_group = robot.main_group_name
        for ik_target_frame in ik_target_frames:
            # IK Query to the planner (Frame to Configuration)
            try:
                # Note: The inverse_kinematics method returns a generator
                joint_positions, joint_names = next(
                    planner.inverse_kinematics(
                        robot,
                        ik_target_frame,
                        start_configuration=robot.zero_configuration(),
                        group=planning_group,
                        options=ik_options,
                    )
                )
            except StopIteration:
                assert False, f"No IK Solution found for frame {ik_target_frame}"

            # Reconstruct the full Configuration from the returned joint positions
            ik_result = robot.zero_configuration()
            for name, value in zip(joint_names, joint_positions):
                ik_result[name] = value

            # FK Query to the planner (Configuration to Frame)
            fk_result = planner.forward_kinematics(robot, ik_result, planning_group)

            # Compare the frames
            validate_ik_with_fk(ik_target_frame, fk_result)


def test_pybullet_ik_fk_agreement_ur5():
    robot = RobotLibrary.ur5(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=0.4, y=0.1, z=0.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.01, -0.01, -0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.01, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, 0.0)))

    _test_pybullet_ik_fk_agreement(robot, ik_target_frames)


def test_pybullet_ik_fk_agreement_ur10e():
    robot = RobotLibrary.ur10e(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=0.4, y=0.1, z=0.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.01, -0.01, -0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.01, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, 0.0)))

    _test_pybullet_ik_fk_agreement(robot, ik_target_frames)


def test_pybullet_ik_fk_agreement_abb_irb4600_40_255():
    robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=1.0, y=0.3, z=1.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, -0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.1, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.3, -0.1, -0.3)))

    _test_pybullet_ik_fk_agreement(robot, ik_target_frames)


##################################################
# Testing IK out of reach for the PyBullet backend
##################################################


def test_pybullet_ik_out_of_reach_ur5():
    robot = RobotLibrary.ur5(load_geometry=True)

    ik_target_frames = []

    ik_target_frames.append(
        Frame(
            Point(x=2.4, y=0.1, z=0.3),
            Vector(x=-1.0, y=0.0, z=0.0),
            Vector(x=0.0, y=0.0, z=1.0),
        )
    )
    ik_target_frames.append(
        Frame(
            Point(x=0.4, y=1.5, z=0.3),
            Vector(x=-1.0, y=0.0, z=0.0),
            Vector(x=0.0, y=0.0, z=1.0),
        )
    )

    # high_accuracy_max_iter is set to 20 to reduce the number of iterations, faster testing time.
    ik_options = {"high_accuracy_max_iter": 20, "high_accuracy": True, "high_accuracy_threshold": 1e-5}

    with PyBulletClient(connection_type="direct") as client:
        robot = client.load_existing_robot(robot)
        planner = PyBulletPlanner(client)
        planning_group = robot.main_group_name
        for ik_target_frame in ik_target_frames:
            # IK Query to the planner (Frame to Configuration)
            try:
                # Note: The inverse_kinematics method returns a generator
                joint_positions, joint_names = next(
                    planner.inverse_kinematics(
                        robot,
                        ik_target_frame,
                        start_configuration=robot.zero_configuration(),
                        group=planning_group,
                        options=ik_options,
                    )
                )
                # An error should be thrown here because the IK target is out of reach
                assert False, f"IK Solution found when there should be none: frame {ik_target_frame}"
            except StopIteration:
                continue
