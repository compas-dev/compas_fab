from __future__ import print_function
import os
import logging
import xml.etree.ElementTree as ET
import binascii

import roslibpy

LOGGER = logging.getLogger('urdf_importer')

from compas.datastructures import Mesh


def check_mesh_class(meshcls):
    """Checks if the passed mesh class has the necessary constructor and methods.
    """
    import compas
    from compas.datastructures import Mesh

    try:
        cm = Mesh.from_obj(compas.get('faces.obj'))
        meshcls(cm)
    except:
        raise TypeError("The class %s cannot be constructed from a %s" % (meshcls, Mesh))

    if not hasattr(meshcls, 'transform'):
        raise TypeError("The class %s has no method named 'transform'" % meshcls)
    
    if not hasattr(meshcls, 'draw'):
        raise TypeError("The class %s has no method named 'draw'" % meshcls)


class UrdfImporter(object):
    """Allows to retrieve the mesh files specified in the robot urdf from the
    ROS file_server and stores it on the local file system.

    It is implemented similar to
    https://github.com/siemens/ros-sharp/blob/master/Libraries/RosBridgeClient/UrdfImporter.cs

    Attributes:
        client (:class:`Ros`): The ROS client
        local_directory (str, optional): A directory to store the files.
            Defaults to ~/robot_description
        robot_name (str or None): The name of the robot, will be read from the
            urdf string
        requested_resource_files (dict): The mesh files from the urdf
        status (dict): To check the status of the process.
    """

    def __init__(self, client=None, local_directory=os.path.join(os.path.expanduser('~'), "robot_description")):
        self.client = client
        self.local_directory = local_directory
        self.robot_name = None
        self.requested_resource_files = {}
        self.status = {"robot_description_received": False,
                       "robot_description_semantic_received": False,
                       "resource_files_received": False,
                       "robot_name_received": False}
    
    @classmethod
    def from_robot_resource_path(cls, path):
        local_directory = os.path.abspath(os.path.join(path, ".."))
        importer = cls(local_directory=local_directory)
        importer.robot_name = os.path.basename(os.path.normpath(path))
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
            self.client.close()
            self.client.terminate()

    def receive_robot_description(self, robot_description):
        robot_name, uris = self.read_robot_name_and_uris_from_urdf(robot_description)
        self.robot_name = robot_name
        self.status.update({"robot_name_received": True})
        # Import resource files
        self.import_resource_files(uris)
        # Save robot_description.urdf
        self.save_robot_description(robot_description)
        # Save robot_description_semantic.urdf
        param = roslibpy.Param(self.client, '/robot_description_semantic')
        param.get(self.save_robot_description_semantic)
        # Update status
        self.check_status()
    
    def get_robot_description_filename(self):
        return os.path.join(self.robot_resource_path, "robot_description.urdf")
    
    def get_robot_description_semantic_filename(self):
        return os.path.join(self.robot_resource_path, "robot_description_semantic.urdf")
    
    def save_robot_description(self, robot_description):
        # Save robot_description.urdf
        filename = self.get_robot_description_filename()
        LOGGER.info("Saving URDF file to %s" % filename)
        self.write_file(filename, robot_description)
        self.status.update({"robot_description_received": True})
        # Update status
        self.check_status()
    
    def save_robot_description_semantic(self, robot_description_semantic):
        # Save robot_description_semantic.urdf
        filename = self.get_robot_description_semantic_filename()
        LOGGER.info("Saving URDF file to %s" % filename)
        self.write_file(filename, robot_description_semantic)
        self.status.update({"robot_description_semantic_received": True})
        # Update status
        self.check_status()

    def load(self): # cannot use 'import' as method name...
        param = roslibpy.Param(self.client, '/robot_description')
        param.get(self.receive_robot_description)

    def receive_resource_file(self, local_filename, resource_file_uri):

        def write_binary_response_to_file(response):
            LOGGER.info("Saving %s to %s" % (resource_file_uri, local_filename))            
            self.write_file(local_filename, binascii.a2b_base64(response.data['value']), 'wb')
            #self.write_file(local_filename, response.data['value'])
            self.update_file_request_status(resource_file_uri)

        service = roslibpy.Service(self.client, "/file_server/get_file",
                                   "file_server/GetBinaryFile")
        service.call(roslibpy.ServiceRequest({'name': resource_file_uri}),
                     write_binary_response_to_file, errback=None)

    def read_robot_name_and_uris_from_urdf(self, robot_description):
        root = ET.fromstring(robot_description)
        robot_name = root.attrib['name']
        uris = [mesh.attrib['filename'] for mesh in root.iter('mesh')
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
            else:
                raise FileNotFoundError("Please convert '%s' into an OBJ file, since DAE is currently not supported yet." % filename)
        elif extension == "obj":
            mesh = Mesh.from_obj(filename)
        elif extension == "stl":
            mesh = Mesh.from_stl(filename)
        else:
            raise ValueError("%s file types not yet supported" % 
                extension.upper())
        
        return meshcls(mesh)
    
    def read_robot_semantics(self):
        semantics = {}

        semantic_filename = self.get_robot_description_semantic_filename()
        tree = ET.parse(semantic_filename)
        root = tree.getroot()

        # 1. Find planning groups
        groups = [group.attrib['name'] for group in root.iter('group')]

        semantics['groups'] = {}

        for group in root.iter('group'):
            chain = group.find('chain')
            if chain != None:
                chain = {'base_link': chain.attrib['base_link'], 'tip_link': chain.attrib['tip_link']}
            else: # get links or joints
                print("no chain")
                #raise NotImplementedError
                #for elem in list(group):
                #    print(elem.tag)
            
            semantics['groups'][group.attrib['name']] = {}
            semantics['groups'][group.attrib['name']]['chain'] = chain

        # 2. Find end-effector (= goal link)
        elem = root.find('end_effector')
        if elem != None:
            semantics['end_effector'] = elem.attrib['parent_link']
        else:
            semantics['end_effector'] = None
            for group in root.iter('group'):
                chain = group.find('chain')
                if chain != None:
                    semantics['end_effector'] = chain.attrib['tip_link']
                    break

        return semantics
        


if __name__ == "__main__":

    class ExampleMesh(object):

        def __init__(self, mesh):
            self.mesh = mesh

        def transform(self, transformation):
            mesh_transform(self.mesh, transformation)
        
        def draw(self):
            return self.mesh

    check_mesh_class(ExampleMesh)
    
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

    ros_client = roslibpy.Ros("127.0.0.1", 9090)

    local_directory = os.path.join(os.path.expanduser('~'), "workspace", "robot_description")
    importer = UrdfImporter(ros_client, local_directory)
    importer.load()
    ros_client.call_later(50, ros_client.close)
    ros_client.call_later(52, ros_client.terminate)
    ros_client.run_forever()
    """
