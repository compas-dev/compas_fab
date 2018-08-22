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

    Some clever text

        resource_path (str): the directory, where the urdf_importer has stored
            the urdf files and the robot mesh files
    """
    def __init__(self, model, resource_path, client=None):
        # it needs a filename because it also sources the meshes from the directory
        # model, urdf_importer, resource_path = None, client = None, viewer={}


        self.model = model
        self.client = client
        self.urdf_importer = UrdfImporter.from_robot_resource_path(resource_path)

        """
        urdf_file = self.urdf_importer.get_robot_description_filename()
        if not os.path.isfile(urdf_file):
            raise ValueError("The file 'robot_description.urdf' is not in resource_path")
        """
        
       
        self.name = self.model.name
        self.semantics = self.urdf_importer.read_robot_semantics()
        # the following would be good to be read from semantics
        self.main_planning_group = self.get_main_planning_group()
        # self.base_link = self.get_base_link()
        # self.ee_link = self.get_ee_link()
        # self.planning_groups = self.get_planning_groups()

        # how is this set = via frame? / property
        self.transformation_RCF_WCF = Transformation()
        self.transformation_WCF_RCF = Transformation()
    
    def set_client(self, client):
        self.client = client

    def set_tool(self, tool):
        raise NotImplementedError

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
        """Returns the current joint state. // or the current "configuraion ?"
        """
        names = self.get_configurable_joint_names(planning_group)
        positions = []
        for joint in self.model.iter_joints():
            if joint.name in names:
                positions.append(joint.position)
        return positions

    def create(self, meshcls):
        self.urdf_importer.check_mesh_class(meshcls)
        self.model.root.create(self.urdf_importer, meshcls, Frame.worldXY())

    def get_planning_groups(self):
        return list(self.semantics['groups'].keys())

    def get_main_planning_group(self):
        # get the group that has the chain with the most links
        main_planning_group = None
        main_planning_group_link_number = 0
        for group_name, v in self.semantics['groups'].items():
            if v['chain'] != None:
                # TODO: Does this really work, since the chain is an iterator
                chain_links_it = self.model.iter_link_chain(v['chain']['base_link'], v['chain']['tip_link'])
                chain_links = list(chain_links_it)
                if len(chain_links) > main_planning_group_link_number:
                    main_planning_group_link_number = len(chain_links)
                    main_planning_group = group_name
        return main_planning_group

    def get_configurable_joint_names(self, planning_group=None):
        """This should be read from robot semantics...
        """
        if not planning_group:
            planning_group = self.main_planning_group

        joint_state_names = []

        chain = self.semantics['groups'][planning_group]['chain']
        if chain == None:
            # return all revoulte joints
            for joint in self.model.iter_joints():
                if joint.type != "fixed":
                    joint_state_names.append(joint.name)
        else:
            chainlinks = self.model.iter_link_chain(chain['base_link'], chain['tip_link'])
            for link in chainlinks:
                for joint in link.joints:
                    if joint.type == "revolute":
                        joint_state_names.append(joint.name)
        return joint_state_names

    def get_end_effector_link_name(self):
        return self.semantics['end_effector']

    def get_end_effector_link(self):
        end_effector_name = self.get_end_effector_link_name()
        for link in self.model.iter_links():
            if link.name == end_effector_name:
                return link
        return None

    def get_end_effector_frame(self):
        end_effector_link = self.get_end_effector_link()
        return end_effector_link.parent_joint.origin.copy()

    def get_base_link_name(self):
        return self.semantics['groups'][self.main_planning_group]['chain']['base_link']

    def get_base_link(self):
        base_link_name = self.get_base_link_name()
        if base_link_name:
            for link in self.model.iter_links():
                if link.name == base_link_name:
                    return link
        else:
            return None

    def get_base_frame(self):
        base_link = self.get_base_link()
        # return the joint that is fixed
        if base_link:
            # TODO: check this! for staubli does not work...
            for joint in base_link.joints:
                if joint.type == "fixed":
                    return joint.origin.copy()
        else:
            return Frame.worldXY()

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
