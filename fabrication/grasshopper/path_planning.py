from __future__ import print_function

import logging
from timeit import default_timer as timer
from compas_fabrication.fabrication.grasshopper import *
from compas_fabrication.fabrication.robots.rfl import Configuration

try:
    import clr
    import rhinoscriptsyntax as rs
    from Rhino.Geometry import Transform, Plane
    from Rhino.Geometry import Mesh as RhinoMesh
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

BUILDING_MEMBER_KEY = 'rfl_building_member'
LOG = logging.getLogger('compas_fabrication.grasshopper.path_planning')


def _transform_to_origin(mesh, xform):
    inverse = clr.StrongBox[Transform]()
    if not xform.TryGetInverse(inverse):
        raise ValueError('Unable to get inverse of matrix')

    mesh.Transform(inverse.Value)

    return mesh


def _create_rhino_mesh(vertices, faces):
    vertices = [vertices[i:i + 3] for i in range(0, len(vertices), 3)]
    faces = [faces[i:i + 3] for i in range(0, len(faces), 3)]
    mesh = RhinoMesh()
    for a, b, c in vertices:
        mesh.Vertices.Add(a, b, c)
    for face in faces:
        mesh.Faces.AddFace(face[0], face[1], face[2])

    return mesh


class PathVisualizer(object):
    """Handles the generation of meshes to visualize a full path plan
    in Rhino/Grasshopper.
    """
    def __init__(self, simulator, robot, building_member=None):
        self.simulator = simulator
        self.robot = robot
        self.building_member = building_member

    def get_frame_meshes(self, path, frame, ctx):
        """Retrieves all meshes required to render a specific frame of a path plan.

        Args:
            path (:obj:`list` of :class:`Configuration`): Represents a collision-free
                path to a goal pose. It is the output of the path planning generated
                by a :class:`Simulator` object.
            frame (:obj:`int`): Frame number to retrieve.
            ctx (:obj:`dict`): A dictionary to keep context. Within Grasshopper, this is
                normally the ``sc.sticky`` object.

        Returns:
            list: list of Rhino meshes that can be used to visualize the selected frame.
        """
        first_start = timer() if self.debug else None
        shape_handles = self.simulator.get_all_visible_handles()
        if self.debug:
            LOG.debug('Execution time: get_all_visible_handles=%.2f', timer() - first_start)

        if 'rfl_meshes' not in ctx:
            ctx['rfl_meshes'] = self._get_rfl_meshes(shape_handles)

        frame_config = path[frame]

        if self.building_member and BUILDING_MEMBER_KEY not in ctx:
            ctx[BUILDING_MEMBER_KEY] = self._get_building_member_info(path[0])

        start = timer() if self.debug else None
        self.simulator.set_robot_config(self.robot, frame_config)
        if self.debug:
            LOG.debug('Execution time: set_robot_config=%.2f', timer() - start)

        start = timer() if self.debug else None
        meshes = []
        mesh_matrices = self.simulator.get_object_matrices(shape_handles)
        for handle, mesh_matrix in mesh_matrices.iteritems():
            mesh = ctx['rfl_meshes'][handle].DuplicateShallow()
            mesh.Transform(xform_from_matrix(mesh_matrix))
            meshes.append(mesh)

        if self.building_member:
            info = ctx[BUILDING_MEMBER_KEY]
            mesh = info['mesh'].DuplicateShallow()
            parent_transform = xform_from_matrix(mesh_matrices[info['parent_handle']])
            relative_transform = info['relative_transform']

            mesh.Transform(Transform.Multiply(parent_transform, relative_transform))
            meshes.append(mesh)

        if self.debug:
            LOG.debug('Execution time: get all transformed meshes=%.2f', timer() - start)
            LOG.debug('Execution time: total=%.2f', timer() - first_start)

        return meshes

    @property
    def debug(self):
        """Indicates whether the path visualizer is in debug mode or not."""
        return self.simulator.debug

    def _get_rfl_meshes(self, shape_handles):
        start = timer() if self.debug else None
        shape_geometry = []
        for handle in shape_handles:
            _, faces, vertices, _, _ = self.simulator.run_child_script('getShapeMesh', [handle], [], [])
            shape_geometry.append((vertices, faces))

        rfl_meshes = {}
        _, _, mesh_matrices, _, _ = self.simulator.run_child_script('getShapeMatrices', shape_handles, [], [])
        for i in range(0, len(mesh_matrices), 12):
            handle = shape_handles[i // 12]
            vertices, faces = shape_geometry[i // 12]
            transform = xform_from_matrix(mesh_matrices[i:i + 12])
            mesh = _transform_to_origin(_create_rhino_mesh(vertices, faces), transform)
            rfl_meshes[handle] = mesh

        if self.debug:
            LOG.debug('Execution time: create RFL meshes at origin=%.2f', timer() - start)

        return rfl_meshes

    def _get_building_member_info(self, gripping_config):
        start = timer() if self.debug else None

        self.simulator.set_robot_config(self.robot, gripping_config)
        mesh = mesh_from_guid(self.building_member)
        handle = self.simulator.add_building_member(self.robot, mesh)
        matrix = self.simulator.get_object_matrices([handle])[handle]

        parent_handle = self.simulator.get_object_handle('customGripper' + self.robot.name)
        _, _, mesh_matrix, _, _ = self.simulator.run_child_script('getShapeMatrixRelative', [handle, parent_handle], [], [])

        relative_transform = xform_from_matrix(mesh_matrix)

        transform = xform_from_matrix(matrix)
        mesh_at_origin = _transform_to_origin(rs.coercemesh(self.building_member), transform)

        if self.debug:
            LOG.debug('Execution time: building member=%.2f', timer() - start)

        return {'mesh': mesh_at_origin,
                'parent_handle': parent_handle,
                'relative_transform': relative_transform}


class InputParameterParser(object):
    """Simplifies some tasks related to parsing Grasshopper input parameters
    for path planning."""

    def compact_list(self, list):
        """Compacts a list filtering all `None` values."""
        return filter(None, list)

    def get_pose(self, plane_or_pose):
        """Gets a vrep-compatible pose from a string containing
        comma-separated floats or from a Rhino/Grasshopper plane.

        Args:
            plane_or_pose (comma-separated :obj:`string` of a Rhino :obj:`Plane`):

        Returns:
            list: list of 12 :obj:`float` values representing a pose.
        """
        if not plane_or_pose:
            return None

        if isinstance(plane_or_pose, basestring):
            return map(float, pose.split(',')) if pose else None
        elif isinstance(plane_or_pose, Plane):
            return vrep_pose_from_plane(plane_or_pose)

    def get_config_or_pose(self, config_values_or_plane):
        """Parses multiple input data types and returns a configuration or a pose.

        Args:
            config_values_or_plane (comma-separated :obj:`string`, or :class:`Configuration` instance,
                or a Rhino :obj:`Plane`).
        """
        if not config_values_or_plane:
            return None

        try:
            return vrep_pose_from_plane(config_values_or_plane)
        except (TypeError, IndexError):
            try:
                if config_values_or_plane.coordinates and config_values_or_plane.joint_values:
                    return config_values_or_plane
            except AttributeError:
                values = map(float, config_values_or_plane.split(','))
                return Configuration.from_degrees_list(values)
