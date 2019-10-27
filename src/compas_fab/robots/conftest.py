import math
import time

import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Scale

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots.ur5 import Robot


@pytest.fixture(autouse=True)
def add_imports(doctest_namespace):
    doctest_namespace["math"] = math
    doctest_namespace["time"] = time
    doctest_namespace["Mesh"] = Mesh
    doctest_namespace["Frame"] = Frame
    doctest_namespace["Scale"] = Scale
    doctest_namespace["compas_fab"] = compas_fab


@pytest.fixture(scope="session", autouse=True)
def connect_to_ros(doctest_namespace):
    client = RosClient()
    client.run()
    robot = Robot(client)

    doctest_namespace["client"] = client
    doctest_namespace["robot"] = robot

    yield

    client.close()
