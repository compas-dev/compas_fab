from __future__ import absolute_import
import os
import math
from compas.datastructures.mesh import Mesh
from compas.geometry.basic import subtract_vectors
from compas_fab.fab.geometry.transformation import transform_xyz
from compas_fab.fab.robots import Robot, BaseConfiguration
from compas_fab.fab.geometry import Frame, Rotation, Transformation
from compas_fab.fab.geometry.helpers import mesh_update_vertices
from compas_fab.fab.robots.ur.kinematics import forward_kinematics, inverse_kinematics


class UR(Robot):
    """The UR robot class.
    """

    d1 = 0
    a2 = 0
    a3 = 0
    d4 = 0
    d5 = 0
    d6 = 0

    shoulder_offset = 0
    elbow_offset = 0

    def __init__(self):
        super(UR, self).__init__()

        d1, a2, a3, d4, d5, d6 = self.params

        # j0 - j5 are the axes around which the joints m0 - m5 rotate, e.g. m0
        # rotates around j0, m1 around j1, etc.
        self.j0 = [(0, 0, 0),                 (0, 0, d1)]
        self.j1 = [(0, 0, d1),                (0, -self.shoulder_offset, d1)]
        self.j2 = [(a2, -self.shoulder_offset-self.elbow_offset, d1), (a2, -self.shoulder_offset, d1)]
        self.j3 = [(a2+a3, 0, d1),            (a2+a3,-d4, d1)]
        self.j4 = [(a2+a3, -d4, d1),          (a2+a3, -d4, d1-d5)]
        self.j5 = [(a2+a3, -d4, d1-d5),       (a2+a3, -d4-d6, d1-d5)]

        # check difference ur5 and ur10!!!
        self.tool0_frame = Frame(self.j5[1], [1,0,0], [0,0,1])
        #self.tool0_frame = Frame(self.j5[1], [-1,0,0], [0,0,-1])
        self.model = self.load_model()

    @property
    def params(self):
        """Get UR specific model parameters.

        Returns:
            (:obj:`list` of :obj:`float`): UR specific model parameters.
        """
        return [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]

    def get_model_path(self):
        raise NotImplementedError

    def load_model(self, xdraw_function=None):
        """Load the geometry (meshes) of the robot.

        Args:
            xdraw_function (function, ): The function to draw the
                meshes in the respective CAD environment. Defaults to None.
        """
        path = self.get_model_path()

        # the links loaded as meshes
        m0 = Mesh.from_obj(os.path.join(path, 'base_and_shoulder.obj'))
        m1 = Mesh.from_obj(os.path.join(path, 'upperarm.obj'))
        m2 = Mesh.from_obj(os.path.join(path, 'forearm.obj'))
        m3 = Mesh.from_obj(os.path.join(path, 'wrist1.obj'))
        m4 = Mesh.from_obj(os.path.join(path, 'wrist2.obj'))
        m5 = Mesh.from_obj(os.path.join(path, 'wrist3.obj'))

        # draw the geometry in the respective CAD environment
        if xdraw_function:
            m0 = xdraw_function(m0)
            m1 = xdraw_function(m1)
            m2 = xdraw_function(m2)
            m3 = xdraw_function(m3)
            m4 = xdraw_function(m4)
            m5 = xdraw_function(m5)

        self.model = [m0, m1, m2, m3, m4, m5]

    def get_forward_transformations(self, configuration):
        """Calculate the transformations according to the configuration.

        Args:
            configuration (:class:`Configuration`): The robot's configuration.

        Returns:
            transformations (:obj:`list` of :class:`Transformation`): The
                transformations for each link.
        """
        q0, q1, q2, q3, q4, q5 = configuration.joint_values
        j0, j1, j2, j3, j4, j5 = self.j0, self.j1, self.j2, self.j3, self.j4, self.j5

        T0 = Rotation.from_axis_and_angle(subtract_vectors(j0[1], j0[0]), q0, j0[1])
        j1 = transform_xyz(j1, T0)
        T1 = Rotation.from_axis_and_angle(subtract_vectors(j1[1], j1[0]), q1, j1[1]) * T0
        j2 = transform_xyz(j2, T1)
        T2 = Rotation.from_axis_and_angle(subtract_vectors(j2[1], j2[0]), q2, j2[1]) * T1
        j3 = transform_xyz(j3, T2)
        T3 = Rotation.from_axis_and_angle(subtract_vectors(j3[1], j3[0]), q3, j3[1]) * T2
        j4 = transform_xyz(j4, T3)
        T4 = Rotation.from_axis_and_angle(subtract_vectors(j4[1], j4[0]), q4, j4[1]) * T3
        j5 = transform_xyz(j5, T4)
        T5 = Rotation.from_axis_and_angle(subtract_vectors(j5[1], j5[0]), q5, j5[1]) * T4

        # now apply the transformation to the base
        T0 = self.transformation_RCS_WCS * T0
        T1 = self.transformation_RCS_WCS * T1
        T2 = self.transformation_RCS_WCS * T2
        T3 = self.transformation_RCS_WCS * T3
        T4 = self.transformation_RCS_WCS * T4
        T5 = self.transformation_RCS_WCS * T5

        return T0, T1, T2, T3, T4, T5

    def get_tool0_transformation(self, T5):
        """Get the transformation to reach tool0_frame.
        """
        return T5 * Transformation.from_frame(self.tool0_frame)

    def get_transformed_tool_frames(self, T5):
        T = self.get_tool0_transformation(T5)
        tool0_frame = Frame.from_transformation(T)
        tcp_frame = Frame.from_transformation(T * self.transformation_tcp_tool0)
        return tool0_frame, tcp_frame

    def get_transformed_model(self, transformations, xtransform_function=None):
        """Get the transformed meshes of the robot model.

        Args:
            transformations (:obj:`list` of :class:`Transformation`): A list of
                transformations to apply on each of the links
            xform_function (function name, ): the name of the function
                used to transform the model. Defaults to None.

        Returns:
            model (:obj:`list` of :class:`Mesh`): The list of meshes in the
                respective class of the CAD environment
        """
        tmodel = []
        if xtransform_function:
            for m, T in zip(self.model, transformations):
                tmodel.append(xtransform_function(m, T, copy=True))
        else:
            for m, T in zip(self.model, transformations):
                mtxyz = transform_xyz(m.xyz, T)
                faces = [m.face_vertices(fkey) for fkey in m.faces()]
                tmodel.append(Mesh.from_vertices_and_faces(mtxyz, faces))
        return tmodel


    def get_transformed_tool_model(self, T5, xtransform_function=None):
        """Get the transformed meshes of the tool model.

        Args:
            T5 (:class:`Transformation`): The transformation of the robot's
                last joint.
            xform_function (function name, ): the name of the function
                used to transform the model. Defaults to None.

        Returns:
            model (:obj:`list` of :class:`Mesh`): The list of meshes in the
                respective class of the CAD environment
        """
        T = self.get_tool0_transformation(T5)
        return self.tool.get_transformed_model(T, xtransform_function)

    def forward_kinematics(self, configuration):
        """Forward kinematics function.

        Args:
            configuration (:class:`BaseConfiguration`): the 6 joint angles in radians

        Returns:
            frame (:class:`Frame`): The tool0 frame in robot coordinate system (RCS).
        """

        return forward_kinematics(configuration.joint_values, self.params)

    def inverse_kinematics(self, tool0_frame_RCS):
        """Inverse kinematics function.
        Args:
            tool0_frame_RCS (:class:`Frame`): The tool0 frame to reach in robot
                coordinate system (RCS).

        Returns:
            configurations (:obj:`list` of :class:`BaseConfiguration`): A list
                of possible configurations.
        """
        solutions = inverse_kinematics(tool0_frame_RCS, self.params)
        configurations = []
        for joint_values in solutions:
            configurations.append(BaseConfiguration.from_joints(joint_values))
        return configurations
