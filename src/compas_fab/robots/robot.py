from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging

import compas.robots.model
from compas.robots import Joint

from compas.geometry import Frame
from compas.geometry import Transformation
from compas.datastructures import Mesh

from .configuration import Configuration
from .semantics import RobotSemantics
from .ros_fileserver_loader import RosFileServerLoader

from compas_fab.artists import BaseRobotArtist

LOGGER = logging.getLogger('compas_fab.robots.robot')

__all__ = [
    'Robot',
]


class Robot(object):
    """Represents a **robot** instance.

    This class binds together several building blocks, such as the robot's descriptive model,
    its semantic information and an instance of a backend client
    into a cohesive programmable interface. This representation builds upon the model
    described in the class :class:`compas.robots.Robot` of the **COMPAS** framework.

    Attributes
    ----------
    model : :class:`compas.robots.Robot`
        The robot model, usually created out of an URDF structure.
    artist : :class:`compas_fab.artists.BaseRobotArtist`
        Instance of the artist used to visualize the robot.
    semantics : :class:`RobotSemantics`, optional
        The semantic model of the robot.
    client : optional
        The backend client to use for communication, e.g. :class:`RosClient`
    name : :obj:`str`
        The name of the robot
    """

    def __init__(self, robot_model, robot_artist, semantics=None, client=None):
        self.model = robot_model
        self.artist = robot_artist
        self.semantics = semantics
        self.client = client  # setter and getter

    @classmethod
    def basic(cls, name, joints=[], links=[], materials=[], **kwargs):
        """Convenience method to create the most basic instance of a robot, based only on a name.

        Parameters
        ----------
        name : str
            Name of the robot

        Returns
        -------
        :class:`Robot`
            Newly created instance of a robot.
        """
        model = compas.robots.model.Robot(name, joints=joints, links=links, materials=materials, **kwargs)
        return cls(model, None)

    @classmethod
    def from_urdf_model(cls, urdf_model, client=None):
        return cls(urdf_model, None, client)

    @classmethod
    def from_urdf_and_srdf_models(cls, urdf_model, srdf_model, client=None):
        return cls(urdf_model, srdf_model, client)

    @classmethod
    def from_resource_path(cls, directory, client=None):
        """Creates a robot from a directory with the necessary resource files.

        The directory must contain a .urdf, a .srdf file and a directory with
        the robot's geometry as indicated in the urdf file.
        """
        urdf_importer = RosFileServerLoader.from_robot_resource_path(directory)
        urdf_file = urdf_importer.urdf_filename
        
        srdf_file = urdf_importer.srdf_filename
        urdf_model = compas.robots.model.Robot.from_urdf_file(urdf_file)
        srdf_model = RobotSemantics.from_srdf_file(srdf_file, urdf_model)
        return cls(urdf_model, None, srdf_model, client)

    @property
    def name(self):
        """Name of the robot, as defined by its model

        Returns
        -------
        str
            Name of the robot.
        """

        return self.model.name

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
            return self.model.get_end_effector_link_name()
        else:
            return self.semantics.get_end_effector_link_name(group)

    def get_end_effector_link(self, group=None):
        name = self.get_end_effector_link_name(group)
        return self.model.get_link_by_name(name)

    def get_end_effector_frame(self, group=None):
        link = self.get_end_effector_link(group)
        return link.parent_joint.origin.copy()

    def get_base_link_name(self, group=None):
        if not self.semantics:
            return self.model.get_base_link_name()
        else:
            return self.semantics.get_base_link_name(group)

    def get_base_link(self, group=None):
        """Returns the origin frame of the robot.
        """
        name = self.get_base_link_name(group)
        return self.model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        # TODO: check this
        link = self.get_base_link(group)
        if link.parent_joint:
            base_frame = link.parent_joint.origin.copy()
        else:
            base_frame = Frame.worldXY()
        if not self.artist:
            base_frame.point *= self._scale_factor
        return base_frame

    def get_configurable_joints(self, group=None):
        """Returns the configurable joints.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.
        """
        if self.semantics:
            if group:
                return self.semantics.get_configurable_joints(group)
            else:
                joints = []
                for group in self.group_names:
                    joints_in_group = self.semantics.get_configurable_joints(group)
                    for joint in joints_in_group:
                        if joint not in joints: # Check to not add double joints
                            joints.append(joint)
                return joints
        else:
            return self.model.get_configurable_joints()

    def get_configurable_joint_names(self, group=None):
        """Returns the configurable joint names.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.name for j in configurable_joints]
    
    def get_configurable_joint_types(self, group=None):
        """Returns the configurable joint types.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joint types of all groups.
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.type for j in configurable_joints]
    
    def get_group_configuration(self, group, full_configuration):
        """Returns the group's configuration.

        Parameters
        ----------
        group : str
            The name of the group. 
        full_configuration (:class:`Configuration`): The configuration for all 
            configurable joints of the robot.
        """
        values = []
        types = []
        group_joint_names = self.get_configurable_joint_names(group)
        for i, name in enumerate(self.get_configurable_joint_names()):
            if name in group_joint_names:
                types.append(full_configuration.types[i])
                values.append(full_configuration.values[i])
        return Configuration(values, types)
    
    def joint_positions_to_configuration(self, joint_positions, group=None):
        """Returns the robot's configuration from the passed joint_positions.
        """
        types = self.get_configurable_joint_types(group)
        if len(types) != len(joint_positions) and group != None:
            types = self.get_configurable_joint_types()
            full_configuration = Configuration(joint_positions, types)
            return self.get_group_configuration(group, full_configuration)
        else:
            return Configuration(joint_positions, types)


    def transformation_RCF_WCF(self, group=None):
        """Returns the transformation matrix from world coordinate system to 
            robot coordinate system.
        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_frame_to_frame(Frame.worldXY(), base_frame)

    def transformation_WCF_RCF(self, group=None):
        """Returns the transformation matrix from robot coordinate system to 
            world coordinate system
        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_frame_to_frame(base_frame, Frame.worldXY())

    def set_RCF(self, robot_coordinate_frame, group=None):
        """Moves the origin frame of the robot to the robot_coordinate_frame.
        """
        # TODO: must be applied to the model, so that base_frame is RCF
        raise NotImplementedError
    
    def get_RCF(self, group=None):
        """Returns the origin frame of the robot.
        """
        return self.get_base_frame(group)

    def represent_frame_in_RCF(self, frame_WCF, group=None):
        """Returns the representation of a frame in the world coordinate frame
        (WCF) in the robot's coordinate frame (RCF).
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF(group))
        return frame_RCF
    
    def represent_frame_in_WCF(self, frame_RCF, group=None):
        """Returns the representation of a frame in the robot's coordinate frame
        (RCF) in the world coordinate frame (WCF).
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF(group))
        return frame_WCF

    def init_configuration(self, group=None):
        """Returns the init joint configuration.
        """
        types = [joint.type for joint in self.get_configurable_joints(group)]
        positions = [0.] * len(types)
        return Configuration(positions, types)

    def get_configuration(self, group=None):
        """Returns the current joint configuration.
        """
        positions = []
        types = []

        for joint in self.get_configurable_joints(group):
            positions.append(joint.position)
            types.append(joint.type)

        return Configuration(positions, types)

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')

    def ensure_semantics(self):
        if not self.semantics:
            raise Exception('This method is only callable once a semantic model is assigned')

    def inverse_kinematics(self, frame_WCF, current_configuration=None, 
                           callback_result=None, group=None):
        """Calculate the robot's inverse kinematic.

        Parameters
        ----------
            frame (:class:`Frame`): The frame to calculate the inverse for
            current_configuration (:class:`Configuration`, optional): If passed,
                the inverse will be calculated such that the calculated joint 
                positions differ the least from the current configuration.
                Defaults to the zero position for all joints.
            callback_result (function, optional): the function to call for the 
                processing the result. Defaults to the print function.
            group (str, optional): The planning group used for calculation.
                Defaults to the robot's main planning group. 
        
        Examples
        --------
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        if not current_configuration:
            joint_positions = [0] * len(joint_names)
        else:
            if len(joint_names) != len(current_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = current_configuration.values
        if not callback_result:
            callback_result = print

        joint_types = self.get_configurable_joint_types(group)
        for i, t in zip(range(len(joint_positions)), joint_types):
            if t == Joint.PRISMATIC or t == Joint.PLANAR:
                joint_positions[i] /= self.scale_factor

        # represent in RCF
        frame_RCF = self.represent_frame_in_RCF(frame_WCF, group)
        frame_RCF.point /= self.scale_factor # must be in meters
    
        self.client.inverse_kinematics(callback_result, frame_RCF, base_link,
                                       group, joint_names, joint_positions)

    def forward_kinematics(self, configuration, callback_result=None, group=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
            configuration (:class:`Configuration`, optional): The configuration
                to calculate the forward kinematic for.
            callback_result (function, optional): the function to call for the 
                processing the result. Defaults to the print function.
            group (str, optional): The planning group used for calculation.
                Defaults to the robot's main planning group. 
        
        Examples
        --------
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        if not callback_result:
            callback_result = print

        joint_names = self.get_configurable_joint_names(group)
        if len(joint_names) != len(configuration.values):
            raise ValueError("Please pass a configuration with %d values" % len(joint_names))

        joint_positions = configuration.values

        joint_types = self.get_configurable_joint_types(group)
        for i, t in zip(range(len(joint_positions)), joint_types):
            if t == Joint.PRISMATIC or t == Joint.PLANAR:
                joint_positions[i] /= self.scale_factor

        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names(group)
        ee_link = self.get_end_effector_link_name(group)
        self.client.forward_kinematics(callback_result, joint_positions, 
                                       base_link, group, joint_names, ee_link)
        

    def compute_cartesian_path(self, frames_WCF, start_configuration, max_step,
                               avoid_collisions=True, callback_result=None, 
                               group=None):
        """Calculates a path defined by frames (Cartesian coordinate system).

        Parameters
        ----------
            frames (:class:`Frame`): The frames of which the path is defined.
            start_configuration (:class:`Configuration`, optional): The robot's
                configuration at the starting position.
            max_step (float): the approximate distance between the calculated 
                points. (Defined in the robot's units)
            avoid_collisions (bool)
            callback_result (function, optional): the function to call for the 
                processing the result. Defaults to the print function.
            group (str, optional): The planning group used for calculation.
                Defaults to the robot's main planning group. 
        
        Examples
        --------
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        if not callback_result:
            callback_result = print
        frames_RCF = []
        for frame_WCF in frames_WCF:
             # represent in RCF
            frame_RCF = self.represent_frame_in_RCF(frame_WCF, group)
            frame_RCF.point /= self.scale_factor
            frames_RCF.append(frame_RCF)
        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        if not start_configuration:
            joint_positions = [0] * len(joint_names)
        else:
            if len(joint_names) != len(start_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = start_configuration.values
        
        ee_link = self.get_end_effector_link_name(group)
        max_step_scaled = max_step/self.scale_factor
        
        self.client.compute_cartesian_path(callback_result, frames_RCF, base_link, 
                                           ee_link, group, joint_names, joint_positions,
                                           max_step_scaled, avoid_collisions)
    
    def motion_plan_goal_frame(self, frame_WCF, start_configuration, 
                    tolerance_position, tolerance_angle, callback_result=None, 
                    group=None, path_constraints=None,
                    trajectory_constraints=None, planner_id=None):
        """Calculates a motion from start_configuration to frame_WCF.

        Parameters
        ----------
            frame_WCF (:class:`Frame`): The goal frame.
            start_configuration (:class:`Configuration`, optional): The robot's
                configuration at the starting position.
            tolerance_position (float): the allowed tolerance to the frame's 
                position. (Defined in the robot's units)
            tolerance_angle (float): the allowed tolerance to the frame's 
                orientation in radians.
            callback_result (function, optional): the function to call for the 
                processing the result. Defaults to the print function.
            group (str, optional): The planning group used for calculation.
                Defaults to the robot's main planning group. 
        
        Examples
        --------
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        if not callback_result:
            callback_result = print

        frame_RCF = self.represent_frame_in_RCF(frame_WCF, group)
        frame_RCF.point /= self.scale_factor

        base_link = self.get_base_link_name(group)
        ee_link = self.get_end_effector_link_name(group)
        joint_names = self.get_configurable_joint_names()

        if not start_configuration:
            joint_positions = [0] * len(joint_names)
        else:
            if len(joint_names) != len(start_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = start_configuration.values
        
        self.client.motion_plan_goal_frame(callback_result, frame_RCF, base_link, ee_link,
                                group, joint_names, joint_positions, 
                                tolerance_position, tolerance_angle, 
                                path_constraints, trajectory_constraints,
                                planner_id)
    
    def motion_plan_goal_configuration(self, goal_configuration, 
                    start_configuration, tolerance, callback_result=None, 
                    group=None, path_constraints=None, 
                    trajectory_constraints=None, planner_id=None):
        """Calculates a motion from start_configuration to end_configuration.

        Parameters
        ----------
            goal_configuration (:class:`Frame`): The group's goal configuration.
            start_configuration (:class:`Configuration`, optional): The robot's
                configuration at the starting position.
            tolerance (float or float list): the allowed tolerance to joints'
                position. 
            callback_result (function, optional): the function to call for the 
                processing the result. Defaults to the print function.
            group (str, optional): The planning group used for calculation.
                Defaults to the robot's main planning group. 
        
        Examples
        --------
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        if not callback_result:
            callback_result = print

        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        joint_positions_goal = goal_configuration.values
        joint_names_goal = self.get_configurable_joint_names(group)

        if type(tolerance) != list:
            tolerances = [tolerance] * len(joint_names_goal)
        else:
            if len(tolerance) != len(joint_names_goal):
                raise ValueError("Please pass %d values for joint tolerances" % len(joint_names_goal))
            tolerances = tolerance

        if not start_configuration:
            joint_positions = [0] * len(joint_names)
        else:
            if len(joint_names) != len(start_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = start_configuration.values
        
        self.client.motion_plan_goal_joint_positions(callback_result,         
                    joint_positions_goal, joint_names_goal, tolerances, 
                    base_link, group, joint_names, joint_positions, 
                    path_constraints, trajectory_constraints, planner_id)
    

    def add_collision_mesh_to_planning_scene(self, mesh):
        self.client.add_collision_mesh_to_planning_scene(mesh)

    def send_frame(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_configuration(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_trajectory(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    @property
    def frames(self):
        frames = self.model.frames
        #[frame.transform(self.transformation_RCF_WCF) for frame in frames]
        return frames

    @property
    def axes(self):
        axes = self.model.axes
        #[axis.transform(self.transformation_RCF_WCF) for axis in axes]
        return axes
    
    def update(self, configuration, collision=True, group=None):
        names = self.get_configurable_joint_names(group)
        self.artist.update(configuration, collision, names)

    def draw_visual(self):
        return self.artist.draw_visual()

    def draw_collision(self):
        return self.artist.draw_collision()

    def draw(self):
        return self.draw_visual()

    def scale(self, factor):
        """Scale the robot.
        """
        if self.artist:
            self.artist.scale(factor)
        else:
            self._scale_factor = factor

    @property
    def scale_factor(self):
        if self.artist:
            return self.artist.scale_factor
        else:
            return self._scale_factor


