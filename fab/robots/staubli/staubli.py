from __future__ import absolute_import
import os
import math
from compas.datastructures.mesh import Mesh
from compas.geometry.basic import subtract_vectors
from compas.geometry.transformations import transform as transform_xyz
from compas_fab.fab.robots import Robot, BaseConfiguration
from compas_fab.fab.geometry import Frame, Rotation, Transformation
from compas_fab.fab.geometry.helpers import mesh_update_vertices
from compas_fab.fab.robots.kinematics import inverse_kinematics_four_point_method


class Staubli(Robot):
    """
    """

    def __init__(self):
        super(Staubli, self).__init__()


    def get_forward_transformations(self, configuration):
        """Calculate the transformations according to the configuration.

        Args:
            configuration (:class:`Configuration`): The robot's configuration.

        Returns:
            transformations (:obj:`list` of :class:`Transformation`): The
                transformations for each link.
        """
        q0, q1, q2, q3, q4, q5 = configuration.joint_values

        ax0, ax1, ax2, ax3, ax4, ax5 = self.axis0, self.axis1, self.axis2, self.axis3, self.axis4, self.axis5

        T0 = Rotation.from_axis_and_angle(subtract_vectors(ax0[1], ax0[0]), q0, ax0[1])
        ax1 = transform_xyz(ax1, T0)
        T1 = Rotation.from_axis_and_angle(subtract_vectors(ax1[1], ax1[0]), q1, ax1[1]) * T0
        ax2 = transform_xyz(ax2, T1)
        T2 = Rotation.from_axis_and_angle(subtract_vectors(ax2[1], ax2[0]), q2, ax2[1]) * T1
        ax3 = transform_xyz(ax3, T2)
        T3 = Rotation.from_axis_and_angle(subtract_vectors(ax3[1], ax3[0]), q3, ax3[1]) * T2
        ax4 = transform_xyz(ax4, T3)
        T4 = Rotation.from_axis_and_angle(subtract_vectors(ax4[1], ax4[0]), q4, ax4[1]) * T3
        ax5 = transform_xyz(ax5, T4)
        T5 = Rotation.from_axis_and_angle(subtract_vectors(ax5[1], ax5[0]), q5, ax5[1]) * T4

        # now apply the transformation to the base
        T0 = self.transformation_RCS_WCS * T0
        T1 = self.transformation_RCS_WCS * T1
        T2 = self.transformation_RCS_WCS * T2
        T3 = self.transformation_RCS_WCS * T3
        T4 = self.transformation_RCS_WCS * T4
        T5 = self.transformation_RCS_WCS * T5

        return [T0, T1, T2, T3, T4, T5]

    def get_transformed_model(self, transformations, transform_function=None):
        """Get the transformed meshes of the robot model.

        Args:
            transformations (:obj:`list` of :class:`Transformation`): A list of
                transformations to apply on each of the links
            xtransform_function (function name, ): the name of the function
                used to transform the model. Defaults to None.

        Returns:
            model (:obj:`list` of :class:`Mesh`): The list of meshes in the
                respective class of the CAD environment
        """
        transformations = [self.transformation_RCS_WCS] + transformations
        tmodel = []
        if transform_function:
            for m, T in zip(self.model, transformations):
                tmodel.append(transform_function(m, T, copy=True))
        else:
            for m, T in zip(self.model, transformations):
                mtxyz = transform_xyz(m.xyz, T)
                faces = [m.face_vertices(fkey) for fkey in m.faces()]
                tmodel.append(Mesh.from_vertices_and_faces(mtxyz, faces))
        return tmodel

    def draw(self, configuration, transform_function=None):
        """Get the transformed meshes of the robot and the tool model.

        Args:
            configuration (:class:`BaseConfiguration`): the 6 joint angles in 
                radians
            xtransform_function (function name, ): the name of the function
                used to transform the model. Defaults to None.

        Returns:
            model (:obj:`list` of :class:`Mesh`): The list of the robot's meshes
                in the respective class of the CAD environment
        """
        transformations = self.get_forward_transformations(configuration)
        robot_model = self.get_transformed_model(transformations, transform_function)
        tool_model = None
        if self.tool:
            tool_model = self.get_transformed_tool_model(transformations[5], transform_function)
        tool0_frame, tcp_frame = self.get_transformed_tool_frames(transformations[5])

        return robot_model, tool_model, tool0_frame, tcp_frame