import compas_fab
import pytest

from compas_fab.backends import PyBulletClient
from compas_robots import RobotModel
from compas_fab.robots import Robot
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotLibrary

from compas_robots.resources import LocalPackageMeshLoader


def test_pybullet_client_connection_direct():
    with PyBulletClient(connection_type="direct") as client:
        assert client.is_connected


def test_pybullet_client_set_robot_from_urdf():
    # Testing workflow of loading robot from URDF
    with PyBulletClient(connection_type="direct") as client:
        urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
        srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")
        robot = Robot.from_urdf(urdf_filename, srdf_filename)
        # Assert that set_robot can only be performed with geometry
        with pytest.raises(Exception):
            robot = client.set_robot(robot)

        # Load robot with geometry
        mesh_folder = compas_fab.get("robot_library/ur5_robot")
        robot = Robot.from_urdf(urdf_filename, srdf_filename, mesh_folder)
        robot = client.set_robot(robot)

        assert isinstance(robot, Robot)
        assert robot.name == "ur5_robot"
        # Check that the RobotModel is present
        assert isinstance(robot.model, RobotModel)
        # Check that the robot have geometry
        robot.ensure_geometry()

        # Check that the robot have semantics
        robot.ensure_semantics()
        link_names = robot.get_link_names_with_collision_geometry()
        assert set(link_names) == set(
            [
                "base_link_inertia",
                "shoulder_link",
                "upper_arm_link",
                "forearm_link",
                "wrist_1_link",
                "wrist_2_link",
                "wrist_3_link",
            ]
        )


def test_pybullet_client_set_robot_from_robot_library():
    # Testing workflow of using robot from RobotLibrary
    with PyBulletClient(connection_type="direct") as client:
        robot = RobotLibrary.ur5(load_geometry=False)
        # Assert that set_robot can only be performed with geometry
        with pytest.raises(Exception):
            robot = client.set_robot(robot)

        # Load robot with geometry
        robot = RobotLibrary.ur5(load_geometry=True)
        robot = client.set_robot(robot)

        assert isinstance(robot, Robot)
        assert robot.name == "ur5_robot"
        # Check that the RobotModel is present
        assert isinstance(robot.model, RobotModel)
        # Check that the robot have geometry
        robot.ensure_geometry()
        # Check that the robot have semantics
        robot.ensure_semantics()


def test_pybullet_client_set_all_robots_from_robot_library():
    # Testing workflow of using robot from RobotLibrary
    with PyBulletClient(connection_type="direct") as client:

        def set_and_check_robot(robot):
            robot = client.set_robot(robot)
            assert isinstance(robot, Robot)
            assert isinstance(robot.model, RobotModel)
            client.remove_robot()

        set_and_check_robot(RobotLibrary.ur5())
        set_and_check_robot(RobotLibrary.ur10e())
        set_and_check_robot(RobotLibrary.panda())
        set_and_check_robot(RobotLibrary.abb_irb120_3_58())
        set_and_check_robot(RobotLibrary.abb_irb4600_40_255())
        set_and_check_robot(RobotLibrary.rfl())


def test_pybullet_client_set_robot_configuration():
    with PyBulletClient(connection_type="direct") as client:
        robot = RobotLibrary.panda(load_geometry=True)
        # Typically user would call planner.set_robot_cell() directly
        robot = client.set_robot(robot)
        client._robot_cell = RobotCell(robot)

        # Set configuration
        configuration = robot.model.zero_configuration()
        client.set_robot_configuration(configuration)
        # Check that the configuration is set
        assert client.get_robot_configuration().close_to(configuration)

        # Try to set the finger position
        configuration["panda_finger_joint1"] = 0.02
        client.set_robot_configuration(configuration)
        # Check that the joint and the mimic joint are set
        joint_ids = [client.robot_joint_puids["panda_finger_joint1"], client.robot_joint_puids["panda_finger_joint2"]]
        joint_values = client._get_joint_positions(joint_ids, client.robot_puid)
        assert joint_values == [0.02, 0.02]


def test_pybullet_client_internal_puids():
    with PyBulletClient(connection_type="direct") as client:
        assert len(client.robot_link_puids) == 0
        assert len(client.robot_joint_puids) == 0

        robot = RobotLibrary.ur5(load_geometry=True)
        robot = client.set_robot(robot)

        # Check that all links and joints are loaded
        link_names_in_model = [l.name for l in robot.model.iter_links()]
        joint_names_in_model = [j.name for j in robot.model.iter_joints()]
        assert set(link_names_in_model) == set(client.robot_link_puids.keys())
        assert set(joint_names_in_model) == set(client.robot_joint_puids.keys())
        assert len(client.robot_link_puids) == len(client.robot_joint_puids) + 1

        # The first link has a puid of -1
        links_in_model = list(robot.model.iter_links())
        first_link_name = links_in_model[0].name
        assert client.robot_link_puids[first_link_name] == -1

        # In Pybullet a joint and its child link share the same puid
        # Assert that this is the case for all joints and links in the robot model
        for link_name, link_puid in client.robot_link_puids.items():
            if link_puid == -1:
                # Skip the base link
                continue
            link = robot.model.get_link_by_name(link_name)
            joint = robot.model.find_parent_joint(link)
            assert client.robot_joint_puids[joint.name] == link_puid


def test_pybullet_client_internal_puids_abb():
    with PyBulletClient(connection_type="direct") as client:
        assert len(client.robot_link_puids) == 0
        assert len(client.robot_joint_puids) == 0

        robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)
        robot = client.set_robot(robot)

        # Check that all links and joints are loaded
        link_names_in_model = [l.name for l in robot.model.iter_links()]
        joint_names_in_model = [j.name for j in robot.model.iter_joints()]
        assert set(link_names_in_model) == set(client.robot_link_puids.keys())
        assert set(joint_names_in_model) == set(client.robot_joint_puids.keys())
        assert len(client.robot_link_puids) == len(client.robot_joint_puids) + 1

        # The first link has a puid of -1
        links_in_model = list(robot.model.iter_links())
        first_link_name = links_in_model[0].name
        assert client.robot_link_puids[first_link_name] == -1

        # In Pybullet a joint and its child link share the same puid
        # Assert that this is the case for all joints and links in the robot model
        for link_name, link_puid in client.robot_link_puids.items():
            if link_puid == -1:
                # Skip the base link
                continue
            link = robot.model.get_link_by_name(link_name)
            joint = robot.model.find_parent_joint(link)
            assert client.robot_joint_puids[joint.name] == link_puid


# TODO: After implementing the stateless backend, we should test methods related to planning scene and scene state management.


def test_pybullet_client_link_names():
    robot = RobotLibrary.ur5()
    print("Print tree of Robot Model:")
    # Start with the root link and iterate through all joints recursively
    for joint in robot.model.joints:
        print("Name:{}, Parent:{}, Child:{}".format(joint.name, joint.parent.link, joint.child.link))

    print("Links in Model:")
    print([l.name for l in robot.model.links])
    print("Joints in Model:")
    print([l.name for l in robot.model.joints])
    print("Base Link in Model:")
    print(robot.model.get_base_link_name())
    print("Link name in Robot with main group:")
    print(robot.get_link_names())
    print("Base Link Name in Robot with main group:")
    print(robot.get_base_link_name())

    with PyBulletClient(connection_type="direct") as client:
        client.set_robot(robot)
        print("Links in Backend:")
        print(client.robot_link_puids)
        print("Joints in Backend:")
        print(client.robot_joint_puids)
        # link_names = client._get(robot)
        # print(link_names)
        # print(robot.get_link_names())


if __name__ == "__main__":
    test_pybullet_client_internal_puids()
