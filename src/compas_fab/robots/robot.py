from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import math
import os

import compas
import compas.robots.model
from compas.geometry import Frame
from compas.geometry.xforms import Transformation

from .configuration import Configuration
from .semantics import RobotSemantics
from .urdf_importer import UrdfImporter

LOGGER = logging.getLogger('compas_fab.robots.robot')

__all__ = [
    'Robot',
]

class Robot(object):
    """Represents a robot based on an URDF model.

    Attributes:
        urdf_model (:class:`compas.robots.model.Robot`): The robot model from URDF structure.
        semantics (:class:`RobotSemantics`, optional): The semantic model of the robot.
        client, optional: The backend client to use for communication, e.g. :class:`RosClient`
        name (str): The name of the robot
    """

    def __init__(self, urdf_model, semantics=None, client=None):

        self.urdf_model = urdf_model
        self.semantics = semantics
        self.client = client # setter and getter
        self.name = self.urdf_model.name

        # TODO: if client is ros client: tell urdf importer...
        # should be corrected by urdf_model
        self.RCF = Frame.worldXY()

    @classmethod
    def from_urdf_model(cls, urdf_model, client=None):
        urdf_importer = UrdfImporter.from_urdf_model(urdf_model)
        return cls(urdf_model, None, client)

    @classmethod
    def from_urdf_and_srdf_models(cls, urdf_model, srdf_model, client=None):
        urdf_importer = UrdfImporter.from_urdf_model(urdf_model)
        return cls(urdf_model, srdf_model, client)

    @classmethod
    def from_resource_path(cls, directory, client=None):
        """Creates a robot from a directory with the necessary resource files.

        The directory must contain a .urdf, a .srdf file and a directory with
        the robot's geometry as indicated in the urdf file.
        """
        urdf_importer = UrdfImporter.from_robot_resource_path(directory)
        urdf_file = urdf_importer.urdf_filename
        srdf_file = urdf_importer.srdf_filename
        urdf_model = compas.robots.model.Robot.from_urdf_file(urdf_file)
        srdf_model = RobotSemantics.from_srdf_file(srdf_file, urdf_model)
        return cls(urdf_model, srdf_model, client)

    @property
    def group_names(self):
        self.ensure_semantics()
        return self.semantics.group_names

    @property
    def main_group_name(self):
        self.ensure_semantics()
        return self.semantics.main_group_name

    def get_end_effector_link_name(self, group=None):
        if not self.semantics:
            return self.urdf_model.get_end_effector_link_name()
        else:
            return self.semantics.get_end_effector_link_name(group)

    def get_end_effector_link(self, group=None):
        name = self.get_end_effector_link_name(group)
        return self.urdf_model.get_link_by_name(name)

    def get_end_effector_frame(self, group=None):
        link = self.get_end_effector_link(group)
        return link.parent_joint.origin.copy()

    def get_base_link_name(self, group=None):
        if not self.semantics:
            return self.urdf_model.get_base_link_name()
        else:
            return self.semantics.get_base_link_name(group)

    def get_base_link(self, group=None):
        name = self.get_base_link_name(group)
        return self.urdf_model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        link = self.get_base_link(group)
        for joint in link.joints:
            if joint.type == "fixed":
                return joint.origin.copy()
        else:
            return Frame.worldXY()

    def get_configurable_joints(self, group=None):
        if self.semantics:
            return self.semantics.get_configurable_joints(group)
        else:
            return self.urdf_model.get_configurable_joints()

    def get_configurable_joint_names(self, group=None):
        if self.semantics:
            return self.semantics.get_configurable_joint_names(group)
        else:
            # passive joints are only defined in the srdf model, so we just get
            # the ones that are configurable
            return self.urdf_model.get_configurable_joint_names()

    @property
    def transformation_RCF_WCF(self):
        # transformation matrix from world coordinate system to robot coordinate system
        return Transformation.from_frame_to_frame(Frame.worldXY(), self.RCF)

    @property
    def transformation_WCF_RCF(self):
         # transformation matrix from robot coordinate system to world coordinate system
        return Transformation.from_frame_to_frame(self.RCF, Frame.worldXY())

    def set_RCF(self, robot_coordinate_frame):
        self.RCF = robot_coordinate_frame

    def get_configuration(self, group=None):
        """Returns the current joint configuration.
        """
        positions = []
        types = []

        for joint in self.get_configurable_joints(group):
            positions.append(joint.position)
            types.append(joint.type)

        return Configuration(positions, types)

    def update(self, configuration, group=None, collision=False):
        """
        """
        names = self.get_configurable_joint_names(group)
        self.urdf_model.update(names, configuration.values, collision)

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')

    def ensure_semantics(self):
        if not self.semantics:
            raise Exception('This method is only callable once a semantic model is assigned')

    def inverse_kinematics(self, frame):
        self.ensure_client()
        raise NotImplementedError
        configuration = self.client.inverse_kinematics(frame)
        return configuration

    def forward_kinematics(self, configuration):
        self.ensure_client()
        raise NotImplementedError

    def compute_cartesian_path(self, frames):
        self.ensure_client()
        raise NotImplementedError

    def send_frame(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_configuration(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_trajectory(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    @property
    def frames(self):
        return self.urdf_model.get_frames(self.transformation_RCF_WCF)

    @property
    def axes(self):
        return self.urdf_model.get_axes(self.transformation_RCF_WCF)

    def draw_visual(self):
        return self.urdf_model.draw_visual(self.transformation_RCF_WCF)

    def draw_collision(self):
        return self.urdf_model.draw_collision(self.transformation_RCF_WCF)

    def draw(self):
        return self.urdf_model.draw()

    def scale(self, factor):
        """Scale the robot.
        """
        self.urdf_model.scale(factor)

    @property
    def scale_factor(self):
        return self.urdf_model.scale_factor


if __name__ == "__main__":

    import os
    import compas.robots.model
    import xml

    path = r"C:\\Users\\gcasas\\eth\\Labs\\robot_description"

    for item in os.listdir(path):
        fullpath = os.path.join(path, item)
        if os.path.isdir(fullpath) and item[0] != ".":

            try:
                robot = Robot.from_resource_path(fullpath)
                # TODO: Replace with the new RobotArtist
                # robot.create(Mesh)
                print("base_link:", robot.get_base_frame())
            except xml.etree.ElementTree.ParseError:
                print(">>>>>>>>>>>>>>>>>> ERROR", item)





            """
            print("base_link_name:", r2.get_base_link_name())
            print("ee_link_name:", r1.get_end_effector_link_name())
            print("ee_link_name:", r2.get_end_effector_link_name())
            print("configurable_joints:", r1.get_configurable_joint_names())
            print("configurable_joints:", r2.get_configurable_joint_names())
            """




    """
    import os
    path = os.path.join(os.path.expanduser('~'), "workspace", "robot_description")
    robot_name = "ur5"
    robot_name = "staubli_tx60l"
    #robot_name = "abb_irb6640_185_280"
    resource_path = os.path.join(path, robot_name)

    filename = os.path.join(resource_path, "robot_description.urdf")
    model = compas.robots.model.Robot.from_urdf_file(filename)

    robot = Robot(model, resource_path, client=None)

    robot.create(Mesh)
    """
