import compas_fab

from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

from compas_fab.backends import RosClient
from compas_fab.robots import Robot

with RosClient() as client:
    # Load the robot model from URDF file
    model = RobotModel.from_urdf_file(compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf"))

    # Load the meshes of the robot links
    loader = LocalPackageMeshLoader(compas_fab.get("robot_library/ur5_robot"), "")
    model.load_geometry(loader)

    # Create a Robot instance from the robot model
    robot = Robot(model, client=client)
    robot.info()

    assert len(robot.get_configurable_joint_names()) == 6
