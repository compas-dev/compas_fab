import math
import os
import time

import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Rotation
from compas.geometry import Scale

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots.tool import Tool
from compas_fab.robots.ur5 import Robot


@pytest.fixture(autouse=True)
def add_imports(doctest_namespace):
    doctest_namespace["os"] = os
    doctest_namespace["math"] = math
    doctest_namespace["time"] = time
    doctest_namespace["Mesh"] = Mesh
    doctest_namespace["Frame"] = Frame
    doctest_namespace["Scale"] = Scale
    doctest_namespace["compas_fab"] = compas_fab
    doctest_namespace["Rotation"] = Rotation
    doctest_namespace["Tool"] = Tool


@pytest.fixture(scope="session", autouse=True)
def connect_to_ros(doctest_namespace):
    client = RosClient()
    client.run()
    robot = Robot(client)

    doctest_namespace["client"] = client
    doctest_namespace["robot"] = robot

    yield

    client.close()
