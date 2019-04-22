from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import binascii
import logging
import os
import tempfile

import roslibpy
from compas.datastructures import Mesh
from compas.datastructures import mesh_transform
from compas.datastructures import meshes_join
from compas.files import XML
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.robots.resources.basic import _get_file_format
from compas.robots.resources.basic import _mesh_import
from compas.utilities import await_callback

LOGGER = logging.getLogger('compas_fab.robots.ros')

__all__ = [
    'RosFileServerLoader',
]


class RosFileServerLoader(object):
    """Allows to retrieve the mesh files specified in the robot model from the
    ROS File Server. Optionally, it stores them on the local file system,
    allowing them to be loaded by local package loaders afterwards.

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
    """

    def __init__(self, ros=None, local_cache=False, local_cache_directory=None):
        self._robot_name = None
        self.schema_prefix = 'package://'
        self.ros = ros
        self.local_cache_directory = None
        self.local_cache_enabled = local_cache

        if self.local_cache_enabled:
            self.local_cache_directory = local_cache_directory or os.path.join(
                os.path.expanduser('~'), 'robot_description')

    @property
    def _robot_resource_path(self):
        if not self._robot_name:
            raise Exception('Robot name is not assigned, make sure you loaded URDF first')

        return os.path.join(self.local_cache_directory, self._robot_name)

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
        param = roslibpy.Param(self.ros, parameter_name)
        urdf = await_callback(param.get)
        self._robot_name = self._read_robot_name(urdf)

        if self.local_cache_enabled:
            self._write_file(self._urdf_filename, urdf)

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
        param = roslibpy.Param(self.ros, parameter_name)
        srdf = await_callback(param.get)

        if self.local_cache_enabled:
            self._write_file(self._srdf_filename, srdf)

        return srdf

    def _read_robot_name(self, robot_description):
        # TODO: Optimize this. We really don't need to parse the full URDF
        # only to read the robot's name (only used for local caching)
        xml = XML.from_string(robot_description)
        return xml.root.attrib['name']

    def _write_file(self, filename, file_contents, mode='w'):
        LOGGER.debug('Saving file to %s', filename)

        dirname = os.path.dirname(filename)

        if not os.path.isdir(dirname):
            os.makedirs(dirname)

        with open(filename, mode) as f:
            f.write(file_contents)

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
        :class:`Mesh`
            Instance of a mesh.
        """

        service = roslibpy.Service(self.ros, '/file_server/get_file', 'file_server/GetBinaryFile')
        request = roslibpy.ServiceRequest(dict(name=url))
        response = await_callback(service.call, 'callback', 'errback', request)

        file_extension = _get_file_format(url)
        file_content = binascii.a2b_base64(response.data['value'])
        if self.local_cache_enabled:
            local_filename = self._local_mesh_filename(url)
        else:
            _, local_filename = tempfile.mkstemp(suffix='.' + file_extension, prefix='ros_fileserver_')

        # Just look away, we're about to do something nasty!
        # namespaces are handled differently between the CLI and CPython
        # XML parsers, so, we just get rid of it for DAE files
        if file_extension == 'dae':
            file_content = file_content.replace(b'xmlns="http://www.collada.org/2005/11/COLLADASchema"', b'')

        # compas.files does not support file-like objects so we need to
        # save the file to disk always. If local caching is enabled,
        # we store it in the cache folder, otherwise, as a temp file.
        self._write_file(local_filename, file_content, 'wb')

        return _fileserver_mesh_import(url, local_filename)

    def _local_mesh_filename(self, url):
        return os.path.abspath(os.path.join(self._robot_resource_path, url[len('package://'):]))


def _dae_mesh_importer(filename):
    """This is a very simple implementation of a DAE/Collada parser.
    It merges all solids of the DAE file into one mesh, because
    several other parts of the framework don't support multi-meshes per file."""
    dae = XML.from_file(filename)
    meshes = []

    for mesh_xml in dae.root.findall('.//mesh'):
        for triangle_set in mesh_xml.findall('triangles'):
            triangle_set_data = triangle_set.find('p').text.split()

            # Parse vertices
            vertices_input = triangle_set.find('input[@semantic="VERTEX"]')
            vertices_link = mesh_xml.find('vertices[@id="{}"]/input'.format(vertices_input.attrib['source'][1:]))
            positions = mesh_xml.find('source[@id="{}"]/float_array'.format(vertices_link.attrib['source'][1:]))
            positions = positions.text.split()

            vertices = list(map(float, positions[i:i + 3]) for i in range(0, len(positions), 3))

            # Parse faces
            faces = list(map(int, triangle_set_data[::2]))  # Ignore normals (ever second item is normal index)
            faces = list(faces[i:i + 3] for i in range(0, len(faces), 3))

            mesh = Mesh.from_vertices_and_faces(vertices, faces)

            meshes.append(mesh)

    combined_mesh = meshes_join(meshes)
    # former DAE files have yaxis and zaxis swapped
    # frame = Frame([0, 0, 0], [1, 0, 0], [0, 0, 1])
    # T = Transformation.from_frame(frame)
    # mesh_transform(combined_mesh, T)
    return combined_mesh


def _fileserver_mesh_import(url, filename):
    """Internal function that adds primitive support for DAE files
    to the _mesh_import function of compas.robots."""
    file_extension = _get_file_format(url)

    if file_extension == 'dae':
        # Magic!
        return _dae_mesh_importer(filename)
    else:
        return _mesh_import(url, filename)


if __name__ == "__main__":

    """
    Start following processes on client side:
    roslaunch YOUR_ROBOT_moveit_config demo.launch rviz_tutorial:=true
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch file_server.launch
    """


    import logging

    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    ros = roslibpy.Ros("127.0.0.1", 9090)

    local_directory = os.path.join(os.path.expanduser('~'), "workspace", "robot_description")
    importer = RosFileServerLoader(ros, local_directory)
    importer.load()
    ros.call_later(50, ros.close)
    ros.call_later(52, ros.terminate)
    ros.run_forever()
