from __future__ import print_function
from compas_fab import get_data
from compas.datastructures.mesh import Mesh
import os
from compas_fab.fab.robots import BaseConfiguration
from compas_fab.fab.robots.staubli import Staubli
from compas_fab.fab.geometry import Frame, Rotation, Transformation

from compas_fab.fab.robots.kinematics.kinematics_4pt import inverse_kinematics_four_point_method

from compas.geometry import subtract_vectors
from compas.geometry import add_vectors

import math


class TX60L(Staubli):
    """
    """

    def __init__(self):
        super(TX60L, self).__init__()

        self.axis0 = [(0.,  0.,   0.),        (0.,  0.,   1.)]
        self.axis1 = [(0.,  0., 375.),        (0.,  1., 375.)]
        self.axis2 = [(0.,  0., 775.),        (0., 20., 775.)]
        self.axis3 = [(0., 20., 775.),        (0., 20., 775. + 450.)]
        self.axis4 = [(0., 20., 775. + 450.), (0.,  0., 775. + 450.)]
        self.axis5 = [(0., 20., 775. + 450.), (0., 20., 775. + 450. + 70.)]

        self.tool0_frame = Frame(self.axis5[1], [0, -1, 0], [1, 0, 0])

        self.p1 = [0.0, 20.0, 375.0]
        self.p2 = [0.0, 0.0, 775.0]
        self.p3 = [450.0, 0.0, 775.0]
        self.p4 = [520.0, 0.0, 775.0]
        self.base_point = [0, 20, 0]

    def get_model_path(self):
        return get_data("robots/staubli/tx60l")

    def load_model(self, draw_function=None):
        """Load the geometry (meshes) of the robot.

        Args:
            draw_function (function, ): The function to draw the
                meshes in the respective CAD environment. Defaults to None.
        """
        path = self.get_model_path()

        # the links loaded as meshes
        m0 = Mesh.from_obj(os.path.join(path, 'base_link.obj'))
        m1 = Mesh.from_obj(os.path.join(path, 'link1.obj'))
        m2 = Mesh.from_obj(os.path.join(path, 'link2.obj'))
        m3 = Mesh.from_obj(os.path.join(path, 'link3.obj'))
        m4 = Mesh.from_obj(os.path.join(path, 'link4.obj'))
        m5 = Mesh.from_obj(os.path.join(path, 'link5.obj'))
        m6 = Mesh.from_obj(os.path.join(path, 'link6.obj'))

        # draw the geometry in the respective CAD environment
        if draw_function:
            m0 = draw_function(m0)
            m1 = draw_function(m1)
            m2 = draw_function(m2)
            m3 = draw_function(m3)
            m4 = draw_function(m4)
            m5 = draw_function(m5)
            m6 = draw_function(m6)

        self.model = [m0, m1, m2, m3, m4, m5, m6]

    
    def get_tool0_transformation(self, T5):
        """Get the transformation to reach tool0_frame.
        """
        return T5 * Transformation.from_frame(self.tool0_frame)

    def get_transformed_tool_frames(self, T5):
        T = self.get_tool0_transformation(T5)
        tool0_frame = Frame.from_transformation(T)
        tcp_frame = Frame.from_transformation(T * self.transformation_tcp_tool0)
        return tool0_frame, tcp_frame

    def get_transformed_tool_model(self, T5, xtransform_function=None):
        """Get the transformed meshes of the tool model.

        Args:
            T5 (:class:`Transformation`): The transformation of the robot's
                last joint.
            xtransform_function (function name, ): the name of the function
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

        target_frame = Frame(add_vectors(tool0_frame_RCS.point, self.base_point), tool0_frame_RCS.xaxis, tool0_frame_RCS.yaxis)

        solutions = inverse_kinematics_four_point_method(self.p1, self.p2, self.p3, self.p4, target_frame, self.base_point)
        solutions = map(list, zip(*solutions))

        configurations = []
        for joint_values in solutions:
            configurations.append(BaseConfiguration.from_joints(joint_values))
        return configurations

if __name__ == "__main__":

    from compas_fab.fab.robots import BaseConfiguration
    from compas.geometry import homogenize
    from compas.geometry import dehomogenize
    from compas.geometry.basic import transpose_matrix
    import time

    def multiply_matrices_xalglib(A, B):
        
        import xalglib
        m = len(A)
        k = len(B)
        n = len(B[0])

        C = [[0 for i in range(n)] for j in range(m)]
        
        # # * M, N, K sizes of op1(A) (which is MxK), op2(B) (which is KxN) and C (which is MxN)

        alpha = 1.0
        beta = 0.0
        ia, ja, optypea = 0, 0, 0
        ib, jb, optypeb = 0, 0, 0
        ic, jc = 0, 0

        return xalglib.rmatrixgemm(m, n, k, alpha, A, ia, ja, optypea, B, ib, jb, optypeb, beta, C, ic, jc)


    def mesh_get_transformed_vertices(mesh, T):
        points = homogenize(mesh.xyz)
        #points = transpose_matrix(multiply_matrices(T, transpose_matrix(points)))
        t0 = time.time()
        #points = multiply_matrices_xalglib(T, transpose_matrix(points))
        points = multiply_matrices_xalglib(T, points)
        print(">>", time.time() - t0)
        return dehomogenize(transpose_matrix(points))


    def mesh_update_vertices(mesh, vertices):
        for i in range(len(vertices)):
            mesh.vertex[i].update({'x': vertices[i][0], 'y': vertices[i][1], 'z': vertices[i][2]})


    robot = TX60L()
    robot.load_model()

    degrees = [30.0, 40.0, 50.0, 60.0, 70.0, 0.0]

    radians = [math.radians(a) for a in degrees]
    configuration = BaseConfiguration.from_joints(radians)

    transformations = robot.get_forward_transformations(configuration)

    #t0 = time.time()
    for T, m in zip(transformations, robot.model[1:]):
        
        vertices = mesh_get_transformed_vertices(m, T.matrix)
        mesh_update_vertices(m, vertices)
    #print(">>", time.time() - t0)
    # ('Rhino:', 0.017974853515625)