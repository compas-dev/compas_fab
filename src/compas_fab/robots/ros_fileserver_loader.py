from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import binascii
import logging
import os

import roslibpy
from compas.datastructures import Mesh
from compas.files.xml import XML
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.geometry.transformations.helpers import mesh_transform

from compas.robots.resources.basic import _get_file_format

LOGGER = logging.getLogger('compas_fab.robots.ros')

__all__ = [
    'RosFileServerLoader',
]


class RosFileServerLoader(object):
    """Allows to retrieve the mesh files specified in the robot urdf from the
    ROS file_server, stores it on the local file system and allows to load the 
    meshes afterwards.

    It is implemented similar to
    https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/UrdfImporter.cs

    Attributes:
        ros (:class:`Ros`): The ROS client
        local_directory (str, optional): A directory to store the files.
            Defaults to ~/robot_description
        robot_name (str or None): The name of the robot, will be read from the
            urdf
        requested_resource_files (dict): The mesh files from the urdf
        status (dict): To check the status of the process.
    """

    def __init__(self, ros=None, local_directory=None):
        self.ros = ros
        self.local_directory = local_directory or os.path.join(os.path.expanduser('~'), 'robot_description')
        self.robot_name = None
        self.requested_resource_files = {}
        self.status = {"robot_description_received": False,
                       "robot_description_semantic_received": False,
                       "resource_files_received": False,
                       "robot_name_received": False}
        
        self.schema_prefix = 'package://'

    @classmethod
    def from_robot_resource_path(cls, path, ros=None):
        local_directory = os.path.abspath(os.path.join(path, ".."))
        importer = cls(ros, local_directory)
        importer.robot_name = os.path.basename(os.path.normpath(path))
        return importer

    @classmethod
    def from_urdf_model(cls, urdf_model, ros=None):
        importer = cls(ros, None)
        importer.robot_name = os.path.basename(urdf_model.name)
        return importer

    @property
    def robot_resource_path(self):
        if self.robot_name:
            return os.path.join(self.local_directory, self.robot_name)
        else:
            return None # or error

    def robot_resource_filename(self, resource_file_uri):
        return os.path.abspath(os.path.join(self.robot_resource_path, resource_file_uri[len('package://'):]))

    def check_status(self):
        if all(self.status.values()):
            self.ros.close()
            self.ros.terminate()

    def receive_robot_description(self, robot_description):
        robot_name, uris = self.read_robot_name_and_uris_from_urdf(robot_description)
        self.robot_name = robot_name
        self.status.update({"robot_name_received": True})
        # Import resource files
        self.import_resource_files(uris)
        # Save robot_description.urdf
        self.save_robot_description(robot_description)
        # Save robot_description_semantic.urdf
        param = roslibpy.Param(self.ros, '/robot_description_semantic')
        param.get(self.save_robot_description_semantic)
        # Update status
        self.check_status()

    @property
    def urdf_filename(self):
        return os.path.join(self.robot_resource_path, "urdf", "robot_description.urdf")

    @property
    def srdf_filename(self):
        return os.path.join(self.robot_resource_path, "robot_description_semantic.srdf")

    def save_robot_description(self, robot_description):
        # Save robot_description.urdf
        filename = self.urdf_filename
        LOGGER.info("Saving URDF file to %s" % filename)
        self.write_file(filename, robot_description)
        self.status.update({"robot_description_received": True})
        # Update status
        self.check_status()

    def save_robot_description_semantic(self, robot_description_semantic):
        # Save robot_description_semantic.urdf
        filename = self.srdf_filename
        LOGGER.info("Saving URDF file to %s" % filename)
        self.write_file(filename, robot_description_semantic)
        self.status.update({"robot_description_semantic_received": True})
        # Update status
        self.check_status()

    def load(self): # cannot use 'import' as method name...
        param = roslibpy.Param(self.ros, '/robot_description')
        param.get(self.receive_robot_description)

    def receive_resource_file(self, local_filename, resource_file_uri):

        def write_binary_response_to_file(response):
            LOGGER.info("Saving %s to %s" % (resource_file_uri, local_filename))
            self.write_file(local_filename, binascii.a2b_base64(response.data['value']), 'wb')
            #self.write_file(local_filename, response.data['value'])
            self.update_file_request_status(resource_file_uri)

        service = roslibpy.Service(self.ros, "/file_server/get_file",
                                   "file_server/GetBinaryFile")
        service.call(roslibpy.ServiceRequest({'name': resource_file_uri}),
                     write_binary_response_to_file, errback=None)

    def read_robot_name_and_uris_from_urdf(self, robot_description):
        xml = XML.from_string(robot_description)
        robot_name = xml.root.attrib['name']
        uris = [mesh.attrib['filename'] for mesh in xml.root.iter('mesh')
            if mesh.attrib['filename'] != '']
        return robot_name, uris

    def import_resource_files(self, uris):
        for resource_file_uri in uris:
            self.requested_resource_files.update({resource_file_uri: False})
            local_filename = self.robot_resource_filename(resource_file_uri)
            if os.path.isfile(local_filename):
                LOGGER.info("Resource file already exists in %s, aborting download." % (local_filename))
                self.update_file_request_status(resource_file_uri)
            else:
                self.receive_resource_file(local_filename, str(resource_file_uri))

    def write_file(self, filename, filecontents, mode='w'):
        dirname = os.path.dirname(filename)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        f = open(filename, mode)
        f.write(filecontents)
        f.close()

    def update_file_request_status(self, resource_file_uri):
        self.requested_resource_files[resource_file_uri] = True
        if all(self.requested_resource_files.values()):
            self.status.update({"resource_files_received": True})
            self.check_status()

    def read_mesh_from_resource_file_uri(self, resource_file_uri, meshcls):
        """Reads the mesh from a file uri and creates a mesh type based on the
        passed mesh class.

        Args:
            resource_file_uri (str): The resourec file starting with package://
            meshcls (:class:): A class that allows to create a custom mesh type
                and is created with a :class:`Mesh`
        """
        filename = self.robot_resource_filename(resource_file_uri)
        return self.read_mesh_from_filename(filename, meshcls)

    def read_mesh_from_filename(self, filename, meshcls):
        if not os.path.isfile(filename):
            raise FileNotFoundError("No such file: '%s'" % filename)
        extension = filename[(filename.rfind(".") + 1):]
        if extension == "dae": # no dae support yet
            #mesh = Mesh.from_dae(filename)
            obj_filename = filename.replace(".dae", ".obj")
            if os.path.isfile(obj_filename):
                mesh = Mesh.from_obj(obj_filename)
                # former DAE files have yaxis and zaxis swapped
                # TODO: already fix in conversion to obj
                frame = Frame([0,0,0], [1,0,0], [0,0,1])
                T = Transformation.from_frame(frame)
                mesh_transform(mesh, T)
            else:
                raise FileNotFoundError("Please convert '%s' into an OBJ file, \
                                         since DAE is currently not supported \
                                         yet." % filename)
        elif extension == "obj":
            mesh = Mesh.from_obj(filename)
        elif extension == "stl":
            mesh = Mesh.from_stl(filename)
        else:
            raise ValueError("%s file types not yet supported" %
                extension.upper())

        return meshcls(mesh)
    
    def _get_local_path(self, url):
        _prefix, path = url.split(self.schema_prefix)
        return os.path.abspath(os.path.join(self.robot_resource_path, path))
        
    def can_load_mesh(self, url):
        """Determine whether this loader can load a given mesh URL.

        Parameters
        ----------
        url : str
            Mesh URL.

        Returns
        -------
        bool
            ``True`` if the URL uses the ``package://` scheme and the package name
            matches the specified in the constructor and the file exists locally,
            otherwise ``False``.
        """
        if not url.startswith(self.schema_prefix):
            return False

        local_file = self._get_local_path(url)
        return os.path.isfile(local_file)

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
        local_file = self._get_local_path(url)
        return _mesh_import(url, local_file)



SUPPORTED_FORMATS = ('obj', 'stl', 'ply', 'dae')

def _mesh_import(url, filename):
    """Internal function to load meshes using the correct loader.

    Name and file might be the same but not always, e.g. temp files."""
    file_extension = _get_file_format(url)

    if file_extension not in SUPPORTED_FORMATS:
        raise NotImplementedError(
            'Mesh type not supported: {}'.format(file_extension))
    
    print(filename)
    
    if file_extension == "dae": # no dae support yet
        #mesh = Mesh.from_dae(filename)
        obj_filename = filename.replace(".dae", ".obj")
        if os.path.isfile(obj_filename):
            mesh = Mesh.from_obj(obj_filename)
            # former DAE files have yaxis and zaxis swapped
            # TODO: already fix in conversion to obj
            frame = Frame([0,0,0], [1,0,0], [0,0,1])
            T = Transformation.from_frame(frame)
            mesh_transform(mesh, T)
            return mesh
        else:
            raise FileNotFoundError("Please convert '%s' into an OBJ file, \
                                        since DAE is currently not supported \
                                        yet." % filename)

    if file_extension == 'obj':
        return Mesh.from_obj(filename)
    elif file_extension == 'stl':
        return Mesh.from_stl(filename)
    elif file_extension == 'ply':
        return Mesh.from_ply(filename)

    raise Exception

if __name__ == "__main__":
    
    """
    Start following processes on client side:
    roslaunch YOUR_ROBOT_moveit_config demo.launch rviz_tutorial:=true
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch file_server.launch
    """

    """
    import logging

    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    ros = roslibpy.Ros("127.0.0.1", 9090)

    local_directory = os.path.join(os.path.expanduser('~'), "workspace", "robot_description")
    importer = UrdfImporter(ros, local_directory)
    importer.load()
    ros.call_later(50, ros.close)
    ros.call_later(52, ros.terminate)
    ros.run_forever()
    """

    path = r"C:\Users\rustr\workspace\robot_description\ur5_with_measurement_tool"
    loader = RosFileServerLoader.from_robot_resource_path(path)

    from compas.robots import Robot

    robot = Robot.from_urdf_file(loader.urdf_filename)
    robot.load_geometry(loader)
