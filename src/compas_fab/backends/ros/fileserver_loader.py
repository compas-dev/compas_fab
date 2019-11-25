from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import binascii
import logging
import os
import tempfile
from collections import OrderedDict

import roslibpy
from compas.datastructures import Mesh
from compas.datastructures import mesh_transform
from compas.files import XML
from compas.geometry import Transformation
from compas.robots.resources.basic import _get_file_format
from compas.robots.resources.basic import _mesh_import
from compas.utilities import geometric_key

LOGGER = logging.getLogger('compas_fab.backends.ros')
TIMEOUT = 10

__all__ = [
    'RosFileServerLoader',
]


def _cache_file_exists(filename):
    return os.path.isfile(filename)


def _read_file(filename, mode='r'):
    LOGGER.debug('Loading file %s from local cache dir', filename)

    with open(filename, mode) as f:
        return f.read()


def _write_file(filename, file_contents, mode='w'):
    LOGGER.debug('Saving file to %s', filename)

    dirname = os.path.dirname(filename)

    if not os.path.isdir(dirname):
        os.makedirs(dirname)

    with open(filename, mode) as f:
        f.write(file_contents)


class RosFileServerLoader(object):
    """Allows to retrieve the mesh files specified in the robot model from the
    ROS File Server. Optionally, it stores them on the local file system,
    allowing for faster re-loads as well as enabling them to be loaded by
    the local package loaders afterwards.

    Parameters
    ----------
    ros : :class:`compas_fab.backends.RosClient`
         The ROS client.
    local_cache : bool
        ``True`` to store a local copy of the ROS files, otherwise ``False``.
        Defaults to ``False``.
    local_cache_directory : str, optional
        Directory name to store the cached files. Only used if
        ``local_cache`` is ``True``. Defaults to ``~/robot_description``.
    precision : float
        Defines precision for importing/loading meshes. Defaults to ``compas.PRECISION``.
    """

    def __init__(self, ros=None, local_cache=False, local_cache_directory=None, precision=None):
        self.robot_name = None
        self.schema_prefix = 'package://'
        self.ros = ros
        self.local_cache_directory = None
        self.local_cache_enabled = local_cache
        self.precision = precision

        if self.local_cache_enabled:
            self.local_cache_directory = local_cache_directory or os.path.join(
                os.path.expanduser('~'), 'robot_description')

    @property
    def _robot_resource_path(self):
        if not self.robot_name:
            raise Exception('Robot name is not assigned, make sure you loaded URDF first')

        if not self.local_cache_directory:
            raise ValueError('local_cache_directory not set')

        return os.path.join(self.local_cache_directory, self.robot_name)

    @property
    def _urdf_filename(self):
        return os.path.join(self._robot_resource_path, 'urdf', 'robot_description.urdf')

    @property
    def _srdf_filename(self):
        return os.path.join(self._robot_resource_path, 'robot_description_semantic.srdf')

    def load_urdf(self, parameter_name='/robot_description'):
        """Loads a URDF model from the specified ROS parameter.

        Parameters
        ----------
        parameter_name : str, optional
            Name of the ROS parameter containing the robot description.
            Defaults to ``/robot_description``.

        Returns
        -------
        str
            URDF model of the robot currently loaded in ROS.
        """
        if self.local_cache_enabled and self.robot_name:
            filename = self._urdf_filename

            if _cache_file_exists(filename):
                return _read_file(filename)

        param = roslibpy.Param(self.ros, parameter_name)
        urdf = param.get(timeout=TIMEOUT)

        self.robot_name = self._read_robot_name(urdf)

        if self.local_cache_enabled:
            # Retrieve filename again, now that robot_name
            # has been loaded from URDF
            _write_file(self._urdf_filename, urdf)

        return urdf

    def load_srdf(self, parameter_name='/robot_description_semantic'):
        """Loads an SRDF model from the specified ROS parameter.

        Parameters
        ----------
        parameter_name : str, optional
            Name of the ROS parameter containing the robot semantics.
            Defaults to ``/robot_description_semantic``.

        Returns
        -------
        str
            SRDF model of the robot currently loaded in ROS.
        """
        if self.local_cache_enabled and self.robot_name:
            filename = self._srdf_filename

            if _cache_file_exists(filename):
                return _read_file(filename)

        param = roslibpy.Param(self.ros, parameter_name)
        srdf = param.get(timeout=TIMEOUT)

        if self.local_cache_enabled:
            _write_file(self._srdf_filename, srdf)

        return srdf

    def _read_robot_name(self, robot_description):
        # TODO: Optimize this. We really don't need to parse the full URDF
        # only to read the robot's name (only used for local caching)
        xml = XML.from_string(robot_description)
        return xml.root.attrib['name']

    def can_load_mesh(self, url):
        """Determine whether this loader can load a given mesh URL.

        Parameters
        ----------
        url : str
            Mesh URL.

        Returns
        -------
        bool
            ``True`` if the URL uses the ``package://` scheme,
            otherwise ``False``.
        """
        return url.startswith(self.schema_prefix)

    def load_mesh(self, url):
        """Loads a mesh from local storage.

        Parameters
        ----------
        url : str
            Mesh location

        Returns
        -------
        :class:`Mesh` or list of :class:`Mesh`
            Instance of a mesh.
        """
        use_local_file = False
        file_extension = _get_file_format(url)

        if self.local_cache_enabled:
            local_filename = self._local_mesh_filename(url)
            use_local_file = _cache_file_exists(local_filename)
        else:
            _, local_filename = tempfile.mkstemp(suffix='.' + file_extension, prefix='ros_fileserver_')

        if not use_local_file:
            service = roslibpy.Service(self.ros, '/file_server/get_file', 'file_server/GetBinaryFile')
            request = roslibpy.ServiceRequest(dict(name=url))
            response = service.call(request, timeout=TIMEOUT)

            file_content = binascii.a2b_base64(response.data['value'])

            # Just look away, we're about to do something nasty!
            # namespaces are handled differently between the CLI and CPython
            # XML parsers, so, we just get rid of it for DAE files
            if file_extension == 'dae':
                file_content = file_content.replace(b'xmlns="http://www.collada.org/2005/11/COLLADASchema"', b'')

            # compas.files does not support file-like objects so we need to
            # save the file to disk always. If local caching is enabled,
            # we store it in the cache folder, otherwise, as a temp file.
            _write_file(local_filename, file_content, 'wb')
        else:
            # Nothing to do here, the file will be read by the mesh importer
            LOGGER.debug('Loading mesh file %s from local cache dir', local_filename)

        return _fileserver_mesh_import(url, local_filename, self.precision)

    def _local_mesh_filename(self, url):
        return os.path.abspath(os.path.join(self._robot_resource_path, url[len('package://'):]))


def _dae_mesh_importer(filename, precision):
    """This is a very simple implementation of a DAE/Collada parser.

    Collada specification: https://www.khronos.org/files/collada_spec_1_5.pdf
    """
    dae = XML.from_file(filename)
    meshes = []
    visual_scenes = dae.root.find('library_visual_scenes')
    materials = dae.root.find('library_materials')
    effects = dae.root.find('library_effects')

    for geometry in dae.root.findall('library_geometries/geometry'):
        mesh_xml = geometry.find('mesh')
        mesh_id = geometry.attrib['id']
        matrix_node = visual_scenes.find('visual_scene/node/instance_geometry[@url="#{}"]/../matrix'.format(mesh_id))
        transform = None

        if matrix_node is not None:
            M = [float(i) for i in matrix_node.text.split()]

            # If it's the identity matrix, then ignore, we don't need to transform it
            if M != [1., 0., 0., 0.,
                     0., 1., 0., 0.,
                     0., 0., 1., 0.,
                     0., 0., 0., 1.]:
                M = M[0:4], M[4:8], M[8:12], M[12:16]
                transform = Transformation.from_matrix(M)

        # primitive elements can be any combination of:
        # lines, linestrips, polygons, polylist, triangles, trifans, tristrips
        # The current implementation only supports triangles and polylist of triangular meshes
        primitive_element_sets = []
        primitive_element_sets.extend(mesh_xml.findall('triangles'))
        primitive_element_sets.extend(mesh_xml.findall('polylist'))

        if len(primitive_element_sets) == 0:
            raise Exception('No primitive elements found (currently only triangles and polylist are supported)')

        for primitive_element_set in primitive_element_sets:
            primitive_tag = primitive_element_set.tag
            primitive_set_data = primitive_element_set.find('p').text.split()

            # Try to retrieve mesh colors
            mesh_colors = {}

            if materials is not None and effects is not None:
                try:
                    instance_effect = None
                    material_id = primitive_element_set.attrib.get('material')
                    primitive_count = int(primitive_element_set.attrib['count'])

                    if material_id is not None:
                        instance_effect = materials.find('material[@id="{}"]/instance_effect'.format(material_id))

                    if instance_effect is not None:
                        instance_effect_id = instance_effect.attrib['url'][1:]
                        colors = effects.findall('effect[@id="{}"]/profile_COMMON/technique/phong/*/color'.format(instance_effect_id))
                        for color_node in colors:
                            rgba = [float(i) for i in color_node.text.split()]
                            mesh_colors['mesh_color.{}'.format(color_node.attrib['sid'])] = rgba
                except Exception:
                    LOGGER.exception('Exception while loading materials, all materials of mesh file %s will be ignored ', filename)

            # Parse vertices
            all_offsets = sorted([int(i.attrib['offset']) for i in primitive_element_set.findall('input[@offset]')])
            if not all_offsets:
                raise Exception('Primitive element node does not contain offset information! Primitive tag={}'.format(primitive_tag))

            vertices_input = primitive_element_set.find('input[@semantic="VERTEX"]')
            vertices_id = vertices_input.attrib['source'][1:]
            vertices_link = mesh_xml.find('vertices[@id="{}"]/input'.format(vertices_id))
            positions = mesh_xml.find('source[@id="{}"]/float_array'.format(vertices_link.attrib['source'][1:]))
            positions = positions.text.split()

            vertices = [[float(p) for p in positions[i:i + 3]] for i in range(0, len(positions), 3)]

            # Parse faces
            # Every nth element is a vertex key, we ignore the rest based on the offsets defined
            # Usually, every second item is the normal, but there can be other items offset in there (vertex tangents, etc)
            skip_step = 1 + all_offsets[-1]

            if primitive_tag == 'triangles':
                vcount = [3] * primitive_count
            elif primitive_tag == 'polylist':
                vcount = [int(v) for v in primitive_element_set.find('vcount').text.split()]

            if len(vcount) != primitive_count:
                raise Exception('Primitive count does not match vertex per face count, vertex input id={}'.format(vertices_id))

            fkeys = [int(f) for f in primitive_set_data[::skip_step]]
            faces = []
            for i in range(primitive_count):
                a = i * vcount[i]
                b = a + vcount[i]
                faces.append(fkeys[a:b])

            # Rebuild vertices and faces using the same logic that other importers
            # use remapping everything based on a selected precision
            index_key = OrderedDict()
            vertex = OrderedDict()

            for i, xyz in enumerate(vertices):
                key = geometric_key(xyz, precision)
                index_key[i] = key
                vertex[key] = xyz

            key_index = {key: index for index, key in enumerate(vertex)}
            index_index = {index: key_index[key] for index, key in iter(index_key.items())}
            vertices = [xyz for xyz in iter(vertex.values())]
            faces = [[index_index[index] for index in face] for face in faces]

            mesh = Mesh.from_vertices_and_faces(vertices, faces)

            if mesh_colors:
                mesh.attributes.update(mesh_colors)

            if transform:
                mesh_transform(mesh, transform)

            meshes.append(mesh)

    return meshes


def _fileserver_mesh_import(url, filename, precision=None):
    """Internal function that adds primitive support for DAE files
    to the _mesh_import function of compas.robots."""
    file_extension = _get_file_format(url)

    if file_extension == 'dae':
        # Magic!
        return _dae_mesh_importer(filename, precision)
    else:
        # TODO: This _mesh_import should also add support for precision
        return _mesh_import(url, filename)


if __name__ == "__main__":
    """
    Start following processes on client side:
    roslaunch YOUR_ROBOT_moveit_config demo.launch rviz_tutorial:=true
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch file_server.launch
    """
    import logging
    from compas.robots import RobotModel
    from compas_fab.backends import RosClient

    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    with RosClient() as ros:
        local_directory = os.path.join(os.path.expanduser('~'), 'workspace', 'robot_description')
        importer = RosFileServerLoader(ros, local_cache=True, local_cache_directory=local_directory)
        importer.robot_name = 'abb_irb1600_6_12'

        urdf = importer.load_urdf()
        srdf = importer.load_srdf()

        model = RobotModel.from_urdf_string(urdf)
        model.load_geometry(importer)

    print(model)
