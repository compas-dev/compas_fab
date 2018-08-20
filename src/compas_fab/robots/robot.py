from __future__ import print_function
import os

import logging
LOGGER = logging.getLogger('compas_fab.robots.robot')

from compas.geometry import Frame
from compas.geometry import add_vectors
from compas.geometry.xforms import Transformation
from compas.geometry.xforms import Rotation
from compas.geometry.xforms import Scale
from compas.geometry.transformations import mesh_transform
from compas.geometry.transformations import mesh_transformed

from compas.robots import Origin as UrdfOrigin
from compas.robots import Visual as UrdfVisual
from compas.robots import Collision as UrdfCollision
from compas.robots import Link as UrdfLink
from compas.robots import Joint as UrdfJoint
from compas.robots import MeshDescriptor as UrdfMeshDescriptor
from compas.robots import Robot as UrdfRobot
from compas.robots.model.geometry import SCALE_FACTOR


#from compas_fab.robots.tool import Tool

from compas_fab.robots.urdf_importer import UrdfImporter

from compas_fab.robots.pose import JointState


class Mesh(object):

    def __init__(self, mesh):
        self.mesh = mesh

    def transform(self, transformation):
        mesh_transform(self.mesh, transformation)

    def draw(self):
        return self.mesh
    
    def set_color(self, color):
        # set colours
        pass


class Robot(object):
    """The robot base class.

    Some clever text

        resource_path (str): the directory, where the urdf_importer has stored
            the urdf files and the robot mesh files
    """
    def __init__(self, resource_path, model=None, client=None):
        # it needs a filename because it also sources the meshes from the directory
        # model, urdf_importer, resource_path = None, client = None, viewer={}

        self.client = client
        self.urdf_importer = UrdfImporter.from_robot_resource_path(resource_path)

        urdf_file = self.urdf_importer.get_robot_description_filename()
        if not os.path.isfile(urdf_file):
            raise ValueError("The file 'robot_description.urdf' is not in resource_path")


       
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

    def set_tool(self, tool):
        raise NotImplementedError

    def set_RCF(self, robot_coordinate_frame):
        raise NotImplementedError
        self.RCF = robot_coordinate_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(Frame.worldXY(), self.RCF)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_RCF_WCF = Transformation.from_frame_to_frame(self.RCF, Frame.worldXY())

    def get_joint_state(self):
        """Returns the current joint state. // or the current "configuraion ?"
        """
        names = self.get_joint_state_names()
        positions = []
        for joint in self.model.iter_joints():
            if joint.name in names:
                positions.append(joint.position)
        return JointState.from_name_and_position(names, positions)

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

    def get_joint_state_names(self, planning_group=None):
        """This should be read from robot semantics...
        """
        if not planning_group:
            planning_group = self.main_planning_group

        joint_state_names = []

        chain = self.semantics['groups'][planning_group]['chain']
        if chain == None:
            # return all revoulte joints
            for joint in self.model.iter_joints():
                if joint.type == "revolute":
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

    def update(self, joint_state):
        """
        """
        # TODO : where to make boundary between message and type
        js = {}
        for k, v in zip(joint_state.name, joint_state.position):
            js[k] = v
        self.model.root.update(js, Transformation(), Transformation())

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')

    def compute_ik(self, pose):
        self.ensure_client()
        raise NotImplementedError

    def compute_cartesian_path(self, poses):
        self.ensure_client()
        raise NotImplementedError

    def send_pose(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_joint_state(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_trajectory(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

class OldRobot(object):
    """Represents the base class for all robots.

    It consists of:
    - a model: meshes
    - a base: describes where the robot is attached to. This can be also a movable base: e.g. linear axis
    - a basis frame, the frame it resides, e.g. Frame.worldXY()
    - a transformation matrix to get coordinates represented in RCS
    - a transformation matrix to get coordinates represented in WCS
    - a tool, the end-effector
    - communication: e.g. delegated by a client instance
    - workspace: brep ?

    self.configuration = [0,0,0,0,0,0]
    self.tcp_frame = tcp_frame
    self.tool0_frame = tool0_frame

    # transform world to robot origin
    self.T_W_R = rg.Transform.PlaneToPlane(Frame.worldXY, self.basis_frame)
    # transform robot to world
    self.T_R_W = rg.Transform.PlaneToPlane(self.basis_frame, Frame.worldXY)
    """

    def __init__(self):

        self.model = []  # a list of meshes
        self.basis_frame = None
        # move to UR !!!!
        self.transformation_RCS_WCS = None
        self.transformation_WCS_RCS = None
        self.set_base(Frame.worldXY())
        self.tool = Tool(Frame.worldXY())
        self.configuration = None

    def set_base(self, base_frame):
        # move to UR !!!! ???
        self.base_frame = base_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_WCS_RCS = Transformation.from_frame_to_frame(Frame.worldXY(), self.base_frame)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_RCS_WCS = Transformation.from_frame_to_frame(self.base_frame, Frame.worldXY())
        # modify joint axis !

    def set_tool(self, tool):
        self.tool = tool

    def get_robot_configuration(self):
        raise NotImplementedError

    @property
    def transformation_tool0_tcp(self):
        return self.tool.transformation_tool0_tcp

    @property
    def transformation_tcp_tool0(self):
        return self.tool.transformation_tcp_tool0

    def forward_kinematics(self, q):
        """Calculate the tcp frame according to the joint angles q.
        """
        raise NotImplementedError

    def inverse_kinematics(self, tcp_frame_RCS):
        """Calculate solutions (joint angles) according to the queried tcp frame
        (in RCS).
        """
        raise NotImplementedError

    def get_frame_in_RCS(self, frame_WCS):
        """Transform the frame in world coordinate system (WCS) into a frame in
        robot coordinate system (RCS), which is defined by the robots' basis frame.
        """
        frame_RCS = frame_WCS.transform(self.transformation_WCS_RCS, copy=True)
        #frame_RCS = frame_WCS.transform(self.transformation_RCS_WCS)
        return frame_RCS

    def get_frame_in_WCS(self, frame_RCS):
        """Transform the frame in robot coordinate system (RCS) into a frame in
        world coordinate system (WCS), which is defined by the robots' basis frame.
        """
        frame_WCS = frame_RCS.transform(self.transformation_RCS_WCS, copy=True)
        return frame_WCS

    def get_tool0_frame_from_tcp_frame(self, frame_tcp):
        """Get the tool0 frame (frame at robot) from the tool frame (frame_tcp).
        """
        T = Transformation.from_frame(frame_tcp)
        return Frame.from_transformation(T * self.transformation_tool0_tcp)

    def get_tcp_frame_from_tool0_frame(self, frame_tool0):
        """Get the tcp frame from the tool0 frame.
        """
        T = Transformation.from_frame(frame_tool0)
        return Frame.from_transformation(T * self.transformation_tcp_tool0)


if __name__ == "__main__":
    """
    base_frame = Frame([-636.57, 370.83, 293.21], [0.00000, -0.54972, -0.83535], [0.92022, -0.32695, 0.21516])
    robot = Robot()
    robot.set_base(base_frame)
    T1 = robot.transformation_WCS_RCS
    T2 = robot.transformation_RCS_WCS
    print(T1 * T2)
    print(robot.transformation_tcp_tool0)
    """
    #filename = r"C:\Users\rustr\robot_description\staubli_tx60l\robot_description.urdf"
    #model = UrdfRobot.from_urdf_file(filename)


    #model = UrdfRobot.from_urdf_files(robot_descriptions, robot_senmantics)
    # have resouce path in model, resolve meshes

    #robot(model, client)
    

    robot = Robot(r"C:\Users\rustr\workspace\robot_description\staubli_tx60l")
    #robot = Robot(r"C:\Users\rustr\workspace\robot_description\ur5")
    #robot = Robot(r"C:\Users\rustr\workspace\robot_description\abb_irb6640_185_280")
    robot.create(Mesh)

    planning_groups = robot.get_planning_groups()
    print("planning_groups", planning_groups)
    main_planning_group = robot.get_main_planning_group()
    print("main_planning_group", main_planning_group)
    for group in planning_groups:
        joint_state_names = robot.get_joint_state_names(group)
        print(group, joint_state_names)

    print("ee_frame", robot.get_end_effector_frame())
    print("base_link_name", robot.get_base_link_name())
    print("base_frame", robot.get_base_frame())

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_positions = [6.254248742364907, -0.06779616254839081, 4.497665741209763, -4.429869574230193, -4.741325546996638, 3.1415926363120015]
    #joint_positions = [6.254248737006559, -5.874885906732766, 4.110686942268209, -1.3773936859827733, -1.5418597546004678, -6.2831853]
    # 400, 100, 400
    #joint_positions = [6.254248737006559, -5.874885906732766, 4.110686942268209, -1.3773936859827733, -1.5418597546004678, -6.2831853]
    joint_positions = [0.5, -0.1, 3, -1, 0.5, 2]

    joint_state = JointState.from_name_and_position(joint_names, joint_positions)

    robot.update(joint_state)


    joint_positions = [6.254248737006559, -5.874885906732766, 4.110686942268209, -1.3773936859827733, -1.5418597546004678, -6.2831853]

    joint_state = JointState.from_name_and_position(joint_names, joint_positions)

    robot.update(joint_state)

    print("==========================")

    for link in robot.model.iter_links():
        for j in link.joints:
            if j.origin:
                print(j.origin)


    visual = robot.draw_visual()
    print(visual)
    collision = robot.draw_collision()
    print(collision)

    for link in robot.model.iter_links():
        for v in link.visual:
            print(link.name, v.origin)

    """
    [[0.0000, 0.0000, 0.0000],  [-1.0000, 0.0000, 0.0000],  [0.0000, -1.0000, 0.0000]]
    [[0.0000, 0.0000, 89.1590],  [1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[0.0000, 135.8500, 89.1590],  [0.0000, 0.0000, -1.0000],  [0.0000, 1.0000, 0.0000]]
    [[425.0000, 16.1500, 89.1590],  [0.0000, 0.0000, -1.0000],  [0.0000, 1.0000, 0.0000]]
    [[817.2500, 16.1500, 89.1590],  [-1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[817.2500, 109.1500, 89.1590],  [-1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[817.2500, 109.1500, -5.4910],  [-1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[817.2500, 191.4500, -5.4910],  [-1.0000, 0.0000, 0.0000],  [0.0000, 0.0000, 1.0000]]

    [[0.0000, 0.0000, 0.0000],  [1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[0.0000, 0.0000, 89.1590],  [0.9996, -0.0289, 0.0000],  [0.0289, 0.9996, 0.0000]]
    [[0.0000, 0.0000, 0.0000],  [-1.0000, 0.0000, 0.0000],  [0.0000, -1.0000, 0.0000]]
    [[3.9305, 135.7931, 89.1590],  [-0.3969, 0.0115, -0.9178],  [0.0289, 0.9996, 0.0000]]
    [[390.3678, 4.8577, -79.5869],  [0.9809, -0.0284, 0.1922],  [0.0289, 0.9996, 0.0000]]
    [[315.0093, 7.0389, 305.3500],  [0.9996, -0.0289, 0.0000],  [0.0289, 0.9996, 0.0000]]
    [[317.7000, 100.0000, 305.3500],  [0.0000, -1.0000, 0.0000],  [1.0000, 0.0000, 0.0000]]
    [[317.7000, 100.0000, 400.0000],  [0.0000, -1.0000, 0.0000],  [1.0000, 0.0000, 0.0000]]
    [[400.0000, 100.0000, 400.0000],  [1.0000, 0.0000, 0.0000],  [0.0000, 1.0000, 0.0000]]
    [[400.0000, 100.0000, 400.0000],  [0.0000, -1.0000, 0.0000],  [0.0000, 0.0000, -1.0000]]
    """

