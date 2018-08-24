from __future__ import print_function

import logging
import os
import math

from compas.geometry import Frame
from compas.geometry import add_vectors
from compas.geometry.transformations import mesh_transform
from compas.geometry.transformations import mesh_transformed
from compas.geometry.xforms import Rotation
from compas.geometry.xforms import Scale
from compas.geometry.xforms import Transformation

from compas.geometry.transformations import mesh_transform
from compas.geometry.transformations import mesh_transformed

from compas.robots import Origin as UrdfOrigin
from compas.robots import Visual as UrdfVisual
from compas.robots import Collision as UrdfCollision
from compas.robots import Joint as UrdfJoint
from compas.robots import Link as UrdfLink
from compas.robots import MeshDescriptor as UrdfMeshDescriptor
from compas.robots import Origin as UrdfOrigin
from compas.robots import Robot as UrdfRobot
from compas.robots import Visual as UrdfVisual
from compas.robots.model.geometry import SCALE_FACTOR


#from compas_fab.robots.tool import Tool

#from compas_fab.robots.pose import JointState
from compas_fab.robots import Configuration
from compas_fab.robots.urdf_importer import UrdfImporter

LOGGER = logging.getLogger('compas_fab.robots.robot')


# RobotArtist?
class Mesh(object):

    def __init__(self, mesh):
        self.mesh = mesh

    def transform(self, transformation):
        mesh_transform(self.mesh, transformation)

    def draw(self):
        return self.mesh
    
    def set_color(self, color_rgba):
        # set colours
        r, g, b, a = color_rgba
        print("color", color_rgba)


class Robot(object):
    """The robot base class.

    Attributes:
        urdf_model (:class:`UrdfRobot`): the model built from URDF structure.
        urdf_importer (class:)

        resource_path (str): the directory, where the urdf_importer has stored
            the urdf files and the robot mesh files
    """
    def __init__(self, urdf_model, urdf_importer, srdf_model=None, client=None):
        # it needs a filename because it also sources the meshes from the directory
        # model, urdf_importer, resource_path = None, client = None, viewer={}

        self.urdf_model = urdf_model
        self.urdf_importer = urdf_importer
        self.srdf_model = srdf_model
        self.client = client
        self.name = self.urdf_model.name

        # TODO: if client is ros client: tell urdf importer...

        # how is this set = via frame? / property
        self.transformation_RCF_WCF = Transformation()
        self.transformation_WCF_RCF = Transformation()
    
    @classmethod
    def from_urdf_model(cls, urdf_model, client=None)
        urdf_importer = UrdfImporter.from_urdf_model(urdf_robot)
        srdf_file = urdf_importer.srdf_filename
        if os.path.isfile(srdf_file):
            srdf_model = SrdfRobot.from_urdf_file(srdf_file, urdf_model)
        else:
            srdf_model = None
        return cls(urdf_model, urdf_importer, srdf_model, client)

    def from_urdf_and_srdf_models(cls, urdf_model, srdf_model, client=None)
        urdf_importer = UrdfImporter.from_urdf_model(urdf_model)
        return cls(urdf_model, urdf_importer, srdf_model, client)
    
    @classmethod
    def from_resource_path(cls, directory, client=None):
        """Creates a robot from a directory with the necessary resource files.

        The directory must contain a .urdf, a .srdf file and a directory with 
        the robot's geometry as indicated in the urdf file.
        """
        urdf_importer = UrdfImporter.from_robot_resource_path(directory)
        urdf_file = urdf_importer.urdf_filename
        srdf_file = urdf_importer.srdf_filename
        urdf_robot = UrdfRobot.from_urdf_file(urdf_file)
        srdf_robot = SrdfRobot.from_urdf_file(srdf_file, urdf_model)
        return cls(urdf_model, urdf_importer, srdf_model, client)
    
    @property
    def group_names(self):
        return self.srdf_model.group_names

    @property
    def main_group_name(self):
        return self.srdf_model.main_group_name
    
    def get_ee_link_name(self, group=None):
        return self.srdf_model.get_ee_link_name(group)
    
    def get_ee_link(self, group=None):
        name = self.get_ee_link_name(group)
        return self.urdf_model.get_link_by_name(name)
    
    def get_ee_frame(self, group=None):
        link = self.get_ee_link()
        return link.parent_joint.origin.copy()
    
    def get_base_link_name(self, group=None):
        return self.srdf_model.get_base_link_name(group)

    def get_base_link(self, group=None):
        name = self.get_base_link_name(group)
        return self.urdf_model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        link = self.get_base_link(group)
        # TODO check this, for staubli is is not correct
        for joint in base_link.joints:
            if joint.type == "fixed":
                return joint.origin.copy()
        else:
            return Frame.worldXY()
    
    def get_configurable_joints(self, group=None):
        if self.srdf_model:
            names = self.srdf_model.get_configurable_joint_names(group)
            return [self.urdf_model.get_joint_by_name(name) for name in names]
        else:
            joints = []
            for joint in self.urdf_model.iter_joints():
                if joint.is_configurable():
                    joints.append(joint)
            return joints

    def get_configurable_joint_names(self, group=None):
        if self.srdf_model:
            return self.srdf_model.get_configurable_joint_names(group)
        else:
            # passive joints are only defined in the srdf model, so we just get
            # the ones that are configurable
            joints = self.get_configurable_joints(group)
            return [j.name for j in joints]

    def set_RCF(self, robot_coordinate_frame):
        raise NotImplementedError
        self.RCF = robot_coordinate_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(Frame.worldXY(), self.RCF)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(self.RCF, Frame.worldXY())

    def get_configuration(self, planning_group=None):
        """Returns the current configuration
        """
        names = self.get_configurable_joint_names(planning_group)
        positions = []
        for joint in self.model.iter_joints():
            # TODO: move this to joint, likewise with setting positions
            if joint.name in names:
                if joint.type in ["revolute", "continuous"]:
                    positions.append(math.degrees(joint.position))
                elif joint.type == ["prismatic", "planar"]:
                    positions.append(joint.position * SCALE_FACTOR)
                else: # floating, fixed
                    positions.append(joint.position)
        return positions

    def get_joint_state(self, planning_group=None):
        # remove joint state and continue use configuration
        """Returns the current joint state. // or the current "configuraion ?"
        """
        names = self.get_configurable_joint_names(planning_group)
        positions = []
        for joint in self.model.iter_joints():
            if joint.name in names:
                positions.append(joint.position)
        return positions

    def create(self, meshcls):
        self.urdf_importer.check_mesh_class(meshcls) # TODO not important if using mesh class
        self.model.root.create(self.urdf_importer, meshcls, Frame.worldXY())

    def get_frames(self):
        return self.model.get_frames()

    def get_axes(self):
        return self.model.get_axes()

    def draw_visual(self):
        return self.model.draw_visual()

    def draw_collision(self):
        return self.model.draw_collision()

    def draw(self):
        return self.model.draw()

    def update(self, configuration, planning_group=None):
        """
        """
        joint_names = self.get_configurable_joint_names(planning_group)
        print('joint_names', joint_names)
        # TODO : where to make boundary between message and type
        js = {}
        for k, v in zip(joint_names, configuration):
            js[k] = v
        self.model.root.update(js, Transformation(), Transformation())

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')
    
    def ensure_srdf_model(self):
        if not self.srdf_robot:
            raise Exception('This method is only callable once a client is assigned')

    def inverse_kinematics(self, frame):
        self.ensure_client()
        raise NotImplementedError
        # return configuration
    
    def forward_kinematics(self, configuration):
        self.ensure_client()
        raise NotImplementedError
        # return frame

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


if __name__ == "__main__":
   
    import os
    path = os.path.join(os.path.expanduser('~'), "workspace", "robot_description")
    robot_name = "ur5"
    robot_name = "staubli_tx60l"
    #robot_name = "abb_irb6640_185_280"
    resource_path = os.path.join(path, robot_name)

    filename = os.path.join(resource_path, "robot_description.urdf")
    model = UrdfRobot.from_urdf_file(filename)

    robot = Robot(model, resource_path, client=None)
    
    robot.create(Mesh)
