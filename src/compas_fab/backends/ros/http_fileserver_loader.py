import logging
import os
import tempfile
import threading
from urllib.parse import urljoin
from urllib.request import urlopen

import roslibpy
from compas.files import XML
from compas_robots.resources import get_file_format

from compas_fab.backends.ros.fileserver_loader import _cache_file_exists
from compas_fab.backends.ros.fileserver_loader import _fileserver_mesh_import
from compas_fab.backends.ros.fileserver_loader import _read_file
from compas_fab.backends.ros.fileserver_loader import _write_file

LOGGER = logging.getLogger("compas_fab.backends.ros")
TIMEOUT = 10

__all__ = [
    "HttpFileServerLoader",
]


class HttpFileServerLoader:
    """Retrieves robot description and mesh files via HTTP and rosbridge topics.

    Designed for the ROS 2 reference backend, where there is no equivalent of the
    ROS 1 ``file_server`` package. Mesh files are fetched over plain HTTP from a
    file server that exposes the ROS share directory (e.g. ``/opt/ros/<distro>/share/``).
    A ``package://my_pkg/path/to/foo.stl`` reference becomes
    ``<base_url>/my_pkg/path/to/foo.stl``. The URDF and SRDF are read from rosbridge
    topics (``/robot_description`` and ``/robot_description_semantic``), which are
    published by ``robot_state_publisher`` and ``move_group`` respectively.

    Parameters
    ----------
    base_url
        Base URL of the HTTP file server, e.g. ``http://localhost:9190``.
    ros
        The ROS client. Required to load URDF/SRDF from topics.
    local_cache
        ``True`` to store a local copy of the fetched files. Defaults to ``False``.
    local_cache_directory
        Directory to store cached files. Only used if ``local_cache`` is ``True``.
        Defaults to ``~/robot_description``.
    timeout
        Network timeout, in seconds, for HTTP requests and topic subscription.
        Defaults to ``10``.
    """

    def __init__(self, base_url, ros=None, local_cache=False, local_cache_directory=None, timeout=TIMEOUT):
        self.base_url = base_url.rstrip("/") + "/"
        self.ros = ros
        self.timeout = timeout
        self.schema_prefix = "package://"
        self.robot_name = None
        self.local_cache_enabled = local_cache
        self.local_cache_directory = None

        if self.local_cache_enabled:
            self.local_cache_directory = local_cache_directory or os.path.join(os.path.expanduser("~"), "robot_description")

    @property
    def _robot_resource_path(self):
        if not self.robot_name:
            raise Exception("Robot name is not assigned, make sure you loaded URDF first")
        if not self.local_cache_directory:
            raise ValueError("local_cache_directory not set")
        return os.path.join(self.local_cache_directory, self.robot_name)

    @property
    def _urdf_filename(self):
        return os.path.join(self._robot_resource_path, "urdf", "robot_description.urdf")

    @property
    def _srdf_filename(self):
        return os.path.join(self._robot_resource_path, "robot_description_semantic.srdf")

    def load_urdf(self, topic="/robot_description", message_type="std_msgs/msg/String"):
        """Load a URDF model from a rosbridge topic.

        ``robot_state_publisher`` publishes the URDF as a latched (``transient_local``)
        ``std_msgs/String`` topic, so late subscribers still receive the description.

        Parameters
        ----------
        topic
            Name of the topic carrying the URDF. Defaults to ``/robot_description``.
        message_type
            ROS message type. Defaults to ``std_msgs/msg/String`` (ROS 2 convention).

        Returns
        -------
        str
            URDF model of the robot currently loaded in ROS.
        """
        if self.local_cache_enabled and self.robot_name:
            filename = self._urdf_filename
            if _cache_file_exists(filename):
                return _read_file(filename)

        urdf = self._read_string_topic(topic, message_type)
        self.robot_name = self._read_robot_name(urdf)

        if self.local_cache_enabled:
            _write_file(self._urdf_filename, urdf)

        return urdf

    def load_srdf(self, topic="/robot_description_semantic", message_type="std_msgs/msg/String"):
        """Load an SRDF model from a rosbridge topic.

        Parameters
        ----------
        topic
            Name of the topic carrying the SRDF. Defaults to ``/robot_description_semantic``.
        message_type
            ROS message type. Defaults to ``std_msgs/msg/String`` (ROS 2 convention).

        Returns
        -------
        str
            SRDF model of the robot currently loaded in ROS.
        """
        if self.local_cache_enabled and self.robot_name:
            filename = self._srdf_filename
            if _cache_file_exists(filename):
                return _read_file(filename)

        srdf = self._read_string_topic(topic, message_type)

        if self.local_cache_enabled:
            _write_file(self._srdf_filename, srdf)

        return srdf

    def _read_string_topic(self, topic_name, message_type):
        if self.ros is None:
            raise ValueError("Pass a `ros` client to load URDF/SRDF from a topic")

        result = []
        event = threading.Event()

        topic = roslibpy.Topic(self.ros, topic_name, message_type)

        def on_message(msg):
            result.append(msg.get("data", ""))
            event.set()

        topic.subscribe(on_message)
        got_message = event.wait(timeout=self.timeout)
        topic.unsubscribe()

        if not got_message:
            raise TimeoutError("Timeout waiting for message on topic {}".format(topic_name))

        return result[0]

    def _read_robot_name(self, robot_description):
        xml = XML.from_string(robot_description)
        return xml.root.attrib["name"]

    def can_load_mesh(self, url):
        """Return ``True`` if the URL uses the ``package://`` scheme."""
        return url.startswith(self.schema_prefix)

    def load_meshes(self, url, precision=None):
        """Load meshes from the given ``package://`` URL via the HTTP file server.

        A single mesh URL can yield multiple meshes depending on the format.

        Parameters
        ----------
        url
            Mesh URL (``package://...``).
        precision
            Precision for parsing geometric data.

        Returns
        -------
        list[:class:`compas.datastructures.Mesh`]
            List of meshes.
        """
        file_extension = get_file_format(url)

        if self.local_cache_enabled:
            local_filename = self._local_mesh_filename(url)
            use_local_file = _cache_file_exists(local_filename)
        else:
            _, local_filename = tempfile.mkstemp(suffix="." + file_extension, prefix="http_fileserver_")
            use_local_file = False

        if not use_local_file:
            http_url = urljoin(self.base_url, url[len(self.schema_prefix) :])
            LOGGER.debug("Fetching mesh from %s", http_url)
            with urlopen(http_url, timeout=self.timeout) as response:
                file_content = response.read()

            # COLLADA namespace handling matches the ROS file server loader.
            if file_extension == "dae":
                file_content = file_content.replace(b'xmlns="http://www.collada.org/2005/11/COLLADASchema"', b"")
                file_content = file_content.replace(b'xmlns="https://www.collada.org/2005/11/COLLADASchema"', b"")

            _write_file(local_filename, file_content, "wb")
        else:
            LOGGER.debug("Loading mesh file %s from local cache dir", local_filename)

        return _fileserver_mesh_import(url, local_filename, precision)

    def _local_mesh_filename(self, url):
        return os.path.abspath(os.path.join(self._robot_resource_path, url[len(self.schema_prefix) :]))
