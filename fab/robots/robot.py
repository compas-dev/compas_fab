from __future__ import print_function
import os

from compas.geometry import Frame
from compas.geometry import add_vectors
from compas.geometry.xforms import Transformation
from compas.geometry.xforms import Rotation
from compas.geometry.xforms import Scale

from compas.datastructures.mesh import Mesh as CMesh
#from .tool import Tool

from compas.robots import Origin as UrdfOrigin
from compas.robots import Visual as UrdfVisual
from compas.robots import Collision as UrdfCollision
from compas.robots import Link as UrdfLink
from compas.robots import Joint as UrdfJoint
from compas.robots import MeshDescriptor as UrdfMeshDescriptor
from compas.robots import Robot as UrdfRobot
from compas.robots.model import SCALE_FACTOR

from compas.geometry.transformations import mesh_transform
from compas.geometry.transformations import mesh_transformed


from compas_fab.fab.robots.urdf_importer import UrdfImporter


# TODO add other types with create as well

class MeshDescriptor(UrdfMeshDescriptor):

    def __init__(self, filename, scale='1.0 1.0 1.0'):
        super(MeshDescriptor, self).__init__(filename, scale)
        self.mesh = None
        self.transformation = Transformation() # the transformation of the mesh
    
    @classmethod
    def from_parent_instance(cls, instance):
        return cls.__init__(instance.filename, instance.scale)

    def create(self, parent): # paremt = Gameobj
        # get robot name?
        # GameObject meshObject = LocateAssetHandler.FindUrdfAsset<GameObject>(self.filename);
        local_filename = os.path.abspath(self.filename.replace('package:/', PATH))
        self.mesh = self.read_mesh_from_filename(local_filename)
        self.set_scale(SCALE_FACTOR)
        #gameObject.transform.SetParentAndAlign(parent.transform);
        return self.mesh 



class Mesh(CMesh):
    """
    """
    def transform(self, transformation):
        mesh_transform(self, transformation)
    
    def transformed(self, transformation):
        return mesh_transformed(self, transformation)

class Robot(object):
    """
        resource_path (str): the directory where all robot mesh files are stored
    """
    def __init__(self, resource_path, client=None):
        # it needs a filename because it also sources the meshes from the directory
        # model, urdf_importer, resource_path = None, client = None, viewer={}
        
        urdf_file = os.path.join(resource_path, "robot_description.urdf")
        if not os.path.isfile(urdf_file):
            raise ValueError("The file 'robot_description.urdf' is not in resource_path")
        self.model = UrdfRobot.from_urdf_file(urdf_file)
        self.urdf_importer = UrdfImporter.from_robot_resource_path(resource_path)
        """
        self.name = name
        self.joints = joints
        self.links = links
        self.materials = materials
        self.attr = kwargs
        self.filename = None
        """

    def create(self, meshcls):
        # create(self, urdf_importer, transform_func)

        #if (UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
        #   Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);

        #self.model.root.create(urdf_importer, transform_func)
        self.model.root.create(self.urdf_importer, meshcls)


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

    def xdraw(configuration, xdraw_function):
        """Draw the robot with the given configuration.
        """
        raise NotImplementedError



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
    robot = Robot(r"C:\Users\rustr\robot_description\staubli_tx60l")
    #robot = Robot(r"C:\Users\rustr\robot_description\ur5")
    robot.create(Mesh)