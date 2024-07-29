import compas_fab
import pytest

from compas_fab.backends import PyBulletClient
from compas_robots import RobotModel
from compas_fab.robots import RobotLibrary

from compas_robots.resources import LocalPackageMeshLoader


def test_pybullet_client_connection_direct():
    with PyBulletClient(connection_type="direct") as client:
        assert client.is_connected


def test_pybullet_client_load_robot():
    with PyBulletClient(connection_type="direct") as client:
        urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
        robot = client.load_robot(urdf_filename)
        assert robot is not None
        assert robot.name == "ur5_robot"
        # Check that the RobotModel is present
        assert isinstance(robot.model, RobotModel)
        # Check that the robot do not have any geometry
        with pytest.raises(Exception):
            robot.ensure_geometry()


def test_pybullet_client_load_robot_with_meshes():
    with PyBulletClient(connection_type="direct") as client:
        urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
        mesh_loader = LocalPackageMeshLoader(compas_fab.get("robot_library/ur5_robot"), "")
        robot = client.load_robot(urdf_filename, [mesh_loader])
        assert robot is not None
        assert robot.name == "ur5_robot"
        assert isinstance(robot.model, RobotModel)
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
        # Check that the robot has geometry
        robot.ensure_geometry()


def test_pybullet_client_load_robot_with_sementics():
    with PyBulletClient(connection_type="direct") as client:
        urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
        mesh_loader = LocalPackageMeshLoader(compas_fab.get("robot_library/ur5_robot"), "")
        robot = client.load_robot(urdf_filename, [mesh_loader])
        srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")
        client.load_semantics(robot, srdf_filename)
        # Check that the robot has geometry
        robot.ensure_geometry()
        # Check that the robot has semantics
        robot.ensure_semantics()


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
    test_pybullet_client_link_names()
