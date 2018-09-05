from __future__ import print_function

import logging
from timeit import default_timer as timer

from compas.datastructures import Mesh
from compas_ghpython.geometry import xform_from_transformation_matrix

from compas_fab.robots import Configuration
from compas_fab.robots import Pose
from compas_fab.backends.vrep.coordinator import SimulationCoordinator

try:
    import clr
    import ghcomp
    import rhinoscriptsyntax as rs
    from Rhino.Geometry import Transform
    from Rhino.Geometry import Mesh as RhinoMesh
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

LOG = logging.getLogger('compas_fab.ghpython.path_planning')


def _trimesh_from_guid(guid, **kwargs):
    """Creates an instance of a :class:`Mesh` class from an identifier
    in Rhino/Grasshopper making sure it is a trimesh.

    This function is almost identical to ``mesh_from_guid`` in the core
    framework, but there were some import issues when used from within
    Grasshopper, but eventually, it should be migrated into the core.
    """
    trimesh = ghcomp.Triangulate(rs.coercemesh(guid))[0]
    vertices = [map(float, vertex) for vertex in rs.MeshVertices(trimesh)]
    faces = map(list, rs.MeshFaceVertices(trimesh))
    faces = [face[: -1] if face[-2] == face[-1] else face for face in faces]
    mesh = Mesh.from_vertices_and_faces(vertices, faces)
    mesh.attributes.update(kwargs)
    return mesh


def vrep_pose_from_plane(plane):
    """Creates a vrep-compatible transformation matrix from a Rhino/Grasshopper
    plane.

    This function might need rework as the source of the 90-deg Y rotation
    need is not entirely clear to me (related to the RFL model mismatch).
    """
    translation_matrix = rs.XformTranslation(((plane[0][0]), (plane[0][1]), plane[0][2]))
    plane_start = rs.PlaneFromFrame(rs.AddPoint(0, 0, 0), rs.AddPoint(1, 0, 0), rs.AddPoint(0, 1, 0))
    plane_end = rs.PlaneFromFrame(rs.AddPoint(0, 0, 0), rs.AddPoint(plane[1][0], (plane[1][1]), plane[1][2]), rs.AddPoint(plane[2][0], plane[2][1], plane[2][2]))
    rotation_matrix = rs.XformRotation1(plane_start, plane_end)
    matrix = rs.XformMultiply(translation_matrix, rotation_matrix)
    return [matrix.M00, matrix.M01, matrix.M02, matrix.M03,
            matrix.M10, matrix.M11, matrix.M12, matrix.M13,
            matrix.M20, matrix.M21, matrix.M22, matrix.M23]


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
    def __init__(self, simulator, robot, building_member=None, building_member_pickup_config=None):
        self.simulator = simulator
        self.robot = robot
        self.building_member = building_member
        self.building_member_pickup_config = building_member_pickup_config

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

        start = timer() if self.debug else None
        self.simulator.set_robot_config(self.robot, frame_config)
        if self.debug:
            LOG.debug('Execution time: set_robot_config=%.2f', timer() - start)

        start = timer() if self.debug else None
        meshes = []
        mesh_matrices = self.simulator.get_object_matrices(shape_handles)
        for handle, mesh_matrix in mesh_matrices.iteritems():
            mesh = ctx['rfl_meshes'][handle].DuplicateShallow()
            mesh.Transform(xform_from_transformation_matrix(mesh_matrix))
            meshes.append(mesh)

        if self.building_member:
            gripping_config = self.building_member_pickup_config if self.building_member_pickup_config else path[0]
            info = self._get_building_member_info(gripping_config)
            mesh = info['mesh'].DuplicateShallow()
            parent_transform = xform_from_transformation_matrix(mesh_matrices[info['parent_handle']])
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
            transform = xform_from_transformation_matrix(mesh_matrices[i:i + 12])
            mesh = _transform_to_origin(_create_rhino_mesh(vertices, faces), transform)
            rfl_meshes[handle] = mesh

        if self.debug:
            LOG.debug('Execution time: create RFL meshes at origin=%.2f', timer() - start)

        return rfl_meshes

    def _get_building_member_info(self, gripping_config):
        start = timer() if self.debug else None

        self.simulator.set_robot_config(self.robot, gripping_config)
        mesh = _trimesh_from_guid(self.building_member)
        handle = self.simulator.add_building_member(self.robot, mesh)
        matrix = self.simulator.get_object_matrices([handle])[handle]

        parent_handle = self.simulator.get_object_handle('customGripper' + self.robot.name)
        _, _, mesh_matrix, _, _ = self.simulator.run_child_script('getShapeMatrixRelative', [handle, parent_handle], [], [])

        relative_transform = xform_from_transformation_matrix(mesh_matrix)

        transform = xform_from_transformation_matrix(matrix)
        mesh_at_origin = _transform_to_origin(rs.coercemesh(self.building_member), transform)

        if self.debug:
            LOG.debug('Execution time: building member=%.2f', timer() - start)

        return {'mesh': mesh_at_origin,
                'parent_handle': parent_handle,
                'relative_transform': relative_transform}


class PathPlanner(object):
    """Provides a simple/compact API to call the path planner from Grasshopper."""

    @classmethod
    def find_path(cls, host='127.0.0.1', port=19997, mode='local', **kwargs):
        """Finds a path for the specified scene description. There is a large number
        of parameters that can be passed as `kwargs`. It can run in two modes: *remote* or
        *local*. In remote mode, the `host` and `port` parameters correspond to a
        simulation coordinator, and in local mode, `host` and `port` correspond to
        a V-REP instance.

        Args:
            host (:obj:`str`): IP address of the service (simulation coordinator in `remote`, V-REP in `local` mode)
            port (:obj:`int`): Port of the service.
            mode (:obj:`str`): Execution mode, either ``local`` or ``remote``.
            kwargs: Keyword arguments.

        Returns:
            list: list of configurations representing a path.
        """
        parser = InputParameterParser()
        options = {'robots': []}
        active_robot = None

        if 'robots' in kwargs:
            for i, settings in enumerate(kwargs['robots']):
                if 'robot' not in settings:
                    raise KeyError("'robot' not found at kwargs['robots'][%d]" % i)

                robot = {'robot': settings['robot']}

                if 'start' in settings:
                    start = parser.get_config_or_pose(settings['start'])
                    if start:
                        robot['start'] = start.to_data()

                if 'goal' in settings:
                    if not active_robot:
                        active_robot = robot
                        goal = parser.get_config_or_pose(settings['goal'])
                        if goal:
                            robot['goal'] = goal.to_data()
                    else:
                        raise ValueError('Multi-move is not (yet) supported. Only one goal can be specified.')

                if 'building_member' in settings:
                    robot['building_member'] = _trimesh_from_guid(settings['building_member']).to_data()

                if 'metric_values' in settings:
                    robot['metric_values'] = map(float, settings['metric_values'].split(','))

                if 'joint_limits' in settings:
                    robot['joint_limits'] = settings['joint_limits']

                options['robots'].append(robot)

        if 'collision_meshes' in kwargs:
            mesh_guids = parser.compact_list(kwargs['collision_meshes'])
            options['collision_meshes'] = map(lambda m: m.to_data(), map(_trimesh_from_guid, mesh_guids))

        options['debug'] = kwargs.get('debug')
        options['trials'] = kwargs.get('trials')
        options['shallow_state_search'] = kwargs.get('shallow_state_search')
        options['optimize_path_length'] = kwargs.get('optimize_path_length')
        options['algorithm'] = kwargs.get('algorithm')
        options['resolution'] = kwargs.get('resolution')

        if mode == 'remote':
            LOG.debug('Running remote path planner executor. Host=%s:%d', host, port)
            return SimulationCoordinator.remote_executor(options, host, port)
        else:
            LOG.debug('Running local path planner executor. Host=%s:%d', host, port)
            return SimulationCoordinator.local_executor(options, host, port)


class InputParameterParser(object):
    """Simplifies some tasks related to parsing Grasshopper input parameters
    for path planning."""

    def compact_list(self, list):
        """Compacts a list filtering all `None` values."""
        return filter(None, list)

    def get_pose(self, plane_or_pose):
        """Gets a pose that is compatible with V-REP from a string containing
        comma-separated floats or from a Rhino/Grasshopper plane.

        Args:
            plane_or_pose (comma-separated :obj:`str` of a Rhino :obj:`Plane`):

        Returns:
            pose: :class:`.Pose` instance representing a transformation matrix.
        """
        if not plane_or_pose:
            return None

        try:
            return Pose.from_list(vrep_pose_from_plane(plane_or_pose))
        except (TypeError, IndexError):
            return Pose.from_list(map(float, plane_or_pose.split(','))) if plane_or_pose else None

    def get_config_or_pose(self, config_values_or_plane):
        """Parses multiple input data types and returns a configuration or a pose
        represented as a transformation matrix.

        Args:
            config_values_or_plane (comma-separated :obj:`string`, or :class:`Configuration` instance,
                or a Rhino :obj:`Plane`).
        """
        if not config_values_or_plane:
            return None

        try:
            return Pose.from_list(vrep_pose_from_plane(config_values_or_plane))
        except (TypeError, IndexError):
            try:
                if config_values_or_plane.external_axes and config_values_or_plane.joint_values:
                    return config_values_or_plane
            except AttributeError:
                values = map(float, config_values_or_plane.split(','))
                return Configuration.from_degrees_list(values)
