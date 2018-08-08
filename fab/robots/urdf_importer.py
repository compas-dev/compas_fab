from __future__ import print_function
import os
import logging
import xml.etree.ElementTree as ET

import roslibpy

LOGGER = logging.getLogger('urdf_importer')


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

    def __init__(self, client, local_directory=os.path.join(os.path.expanduser('~'), "robot_description")):
        self.client = client
        self.local_directory = local_directory
        self.robot_name = None
        self.requested_resource_files = {}
        self.status = {"robot_description_received": False,
                       "resource_files_received": False,
                       "robot_name_received": False}

    def check_status(self):
        if all(self.status.values()):
            self.client.close()
            self.client.terminate()

    def receive_robot_description(self, robot_description):
        robot_name, uris = self.read_robot_name_and_uris_from_urdf(robot_description)
        self.robot_name = robot_name
        self.status.update({"robot_name_received": True})
        self.import_resource_files(uris)
        filename = os.path.join(self.local_directory, self.robot_name, "robot_description.urdf")
        LOGGER.info("Saving URDF file to %s" % filename)
        self.write_file(filename, robot_description)
        self.status.update({"robot_description_received": True})
        self.check_status()

    def load(self):
        param = roslibpy.Param(self.client, '/robot_description')
        param.get(self.receive_robot_description)

    def receive_resource_file(self, local_filename, resource_file_uri):

        def write_binary_response_to_file(response):
            LOGGER.info("Saving %s to %s" % (resource_file_uri, local_filename))
            self.write_file(local_filename, response.data['value'])
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
            local_filename = os.path.abspath(os.path.join(self.local_directory,
                self.robot_name, resource_file_uri[len('package://'):]))
            self.receive_resource_file(local_filename, str(resource_file_uri))

    def write_file(self, filename, filecontents):
        dirname = os.path.dirname(filename)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        f = open(filename, 'w')
        f.write(filecontents)
        f.close()

    def update_file_request_status(self, resource_file_uri):
        self.requested_resource_files[resource_file_uri] = True
        if all(self.requested_resource_files.values()):
            self.status.update({"resource_files_received": True})
            self.check_status()


if __name__ == "__main__":

    import logging

    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    """
    Start following processes on client side:
    roslaunch YOUR_ROBOT_moveit_config demo.launch rviz_tutorial:=true
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch file_server.launch
    """
    ros_client = roslibpy.Ros("127.0.0.1", 9090)

    importer = UrdfImporter(ros_client)
    importer.load()
    ros_client.call_later(50, ros_client.close)
    ros_client.call_later(52, ros_client.terminate)
    ros_client.run_forever()
