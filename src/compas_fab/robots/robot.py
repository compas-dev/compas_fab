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

#from compas_fab.robots.tool import Tool

#from compas_fab.robots.pose import JointState
from compas_fab.robots import Configuration
from compas_fab.robots.urdf_importer import UrdfImporter
from compas_fab.robots.backends.ros import SrdfRobot

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
    """Represents a robot based on an URDF model.

    Attributes:
        urdf_model (:class:`UrdfRobot`): The model built from URDF structure.
        urdf_importer (class:`UrdfImporter`): The importer for loading the meshes.
        srdf_model (class:`SrdfRobot`, optional): The SRDF model.
        client, optional: The client for communication, i.e. class:`Ros`
        name (str): The name of the robot 
    """
    def __init__(self, urdf_model, urdf_importer, srdf_model=None, client=None):

        self.urdf_model = urdf_model
        self.urdf_importer = urdf_importer
        self.srdf_model = srdf_model
        self.client = client # setter and getter
        self.name = self.urdf_model.name

        # TODO: if client is ros client: tell urdf importer...

        # how is this set = via frame? / property
        self.transformation_RCF_WCF = Transformation()
        self.transformation_WCF_RCF = Transformation()
    
    @classmethod
    def from_urdf_model(cls, urdf_model, client=None):
        urdf_importer = UrdfImporter.from_urdf_model(urdf_model)
        return cls(urdf_model, urdf_importer, None, client)

    @classmethod
    def from_urdf_and_srdf_models(cls, urdf_model, srdf_model, client=None):
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
        urdf_model = UrdfRobot.from_urdf_file(urdf_file)
        srdf_model = SrdfRobot.from_srdf_file(srdf_file, urdf_model)
        return cls(urdf_model, urdf_importer, srdf_model, client)
    
    @property
    def group_names(self):
        self.ensure_srdf_model()
        return self.srdf_model.group_names

    @property
    def main_group_name(self):
        self.ensure_srdf_model()
        return self.srdf_model.main_group_name
    
    def get_end_effector_link_name(self, group=None):
        if not self.srdf_model:
            return self.urdf_model.get_end_effector_link_name()
        else:
            return self.srdf_model.get_end_effector_link_name(group)
    
    def get_end_effector_link(self, group=None):
        name = self.get_end_effector_link_name(group)
        return self.urdf_model.get_link_by_name(name)
    
    def get_end_effector_frame(self, group=None):
        link = self.get_end_effector_link()
        return link.parent_joint.origin.copy()
    
    def get_base_link_name(self, group=None):
        if not self.srdf_model:
            return self.urdf_model.get_base_link_name()
        else:
            return self.srdf_model.get_base_link_name(group)

    def get_base_link(self, group=None):
        name = self.get_base_link_name(group)
        return self.urdf_model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        link = self.get_base_link(group)
        # TODO check this, for staubli this is not correct
        for joint in link.joints:
            if joint.type == "fixed":
                return joint.origin.copy()
        else:
            return Frame.worldXY()
    
    def get_configurable_joints(self, group=None):
        if self.srdf_model:
            return self.srdf_model.get_configurable_joints(group)
        else:
            return self.urdf_model.get_configurable_joints()

    def get_configurable_joint_names(self, group=None):
        if self.srdf_model:
            return self.srdf_model.get_configurable_joint_names(group)
        else:
            # passive joints are only defined in the srdf model, so we just get
            # the ones that are configurable
            return self.urdf_model.get_configurable_joint_names()

    def set_RCF(self, robot_coordinate_frame):
        raise NotImplementedError
        self.RCF = robot_coordinate_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(Frame.worldXY(), self.RCF)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(self.RCF, Frame.worldXY())

    def get_configuration(self, group=None):
        """Returns the current joint configuration.
        """
        positions = []
        types = []
        
        for joint in self.get_configurable_joints(group):
            positions.append(joint.position)
            types.append(joint.type)

        return Configuration(positions, types)

    def create(self, meshcls):
        """Loades and creates the meshes with the passed mesh class.
        """
        self.urdf_importer.check_mesh_class(meshcls) # TODO not necessary if using mesh artist
        self.urdf_model.create(self.urdf_importer, meshcls)

    def update(self, configuration, group=None, collision=False):
        """
        """
        names = self.get_configurable_joint_names(group)
        self.urdf_model.update(names, configuration.values, collision)

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')
    
    def ensure_srdf_model(self):
        if not self.srdf_model:
            raise Exception('This method is only callable once a srdf model is assigned')

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
    def frames(self): # get?
        return self.urdf_model.frames

    @property
    def axes(self): # get?
        return self.urdf_model.axes

    def draw_visual(self):
        return self.urdf_model.draw_visual()

    def draw_collision(self):
        return self.urdf_model.draw_collision()

    def draw(self):
        return self.urdf_model.draw()
    
    def scale(self, factor):
        """Scale the robot.
        """
        self.urdf_model.scale(factor)


if __name__ == "__main__":
   
    import os
    from compas.robots import Robot as UrdfRobot

    path = r"C:\Users\rustr\workspace\robot_description"

    for item in os.listdir(path):
        fullpath = os.path.join(path, item)
        if os.path.isdir(fullpath) and item[0] != ".":
            urdf_file = os.path.join(fullpath, 'robot_description.urdf')
            srdf_file = os.path.join(fullpath, 'robot_description_semantic.srdf')

            if item != "panda":
                continue
        
            urdf_model = UrdfRobot.from_urdf_file(urdf_file)
            srdf_model = SrdfRobot.from_srdf_file(srdf_file, urdf_model)

            #r1 = Robot.from_urdf_model(urdf_model)
            #r2 = Robot.from_urdf_and_srdf_models(urdf_model, srdf_model)
            robot = Robot.from_resource_path(fullpath)
            robot.create(Mesh)

            """
            print("base_link_name:", r1.get_base_link_name())
            print("base_link_name:", r2.get_base_link_name())
            print("ee_link_name:", r1.get_end_effector_link_name())
            print("ee_link_name:", r2.get_end_effector_link_name())
            print("configurable_joints:", r1.get_configurable_joint_names())
            print("configurable_joints:", r2.get_configurable_joint_names())
            """

            for joint in robot.get_configurable_joints():
                print(joint.name)
                print(joint.origin)
                print(joint.axis)
                print()

            """

            r3.create(Mesh)
            for f in r3.frames:
                print(frame)

            r3.scale(1000.)

            for f in r3.frames:
                print(frame)

            configuration = Configuration.from_revolute_values([0, 90, 90, 45, 90, 0])
            r3.update(configuration)
            print(r3.get_configuration())
            frames = r3.frames
            for frame in frames:
                print(frame)
            print()
            
            
            break
            """

    """
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
    """
