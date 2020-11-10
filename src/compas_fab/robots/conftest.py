import math
import os
import time

import pytest
import compas
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Rotation
from compas.geometry import Scale
from compas.geometry import allclose

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
    doctest_namespace["compas"] = compas
    doctest_namespace["compas_fab"] = compas_fab
    doctest_namespace["allclose"] = allclose
    doctest_namespace["RosClient"] = RosClient
    doctest_namespace["Rotation"] = Rotation
    doctest_namespace["Tool"] = Tool


@pytest.fixture(scope='function', autouse=True)
def connect_to_ros(request, doctest_namespace):
    if request.module.__name__ == 'compas_fab.robots.robot':
        doctest_namespace["robot"] = Robot()
        yield
    elif request.module.__name__ == 'compas_fab.robots.planning_scene':
        with RosClient() as client:
            robot = Robot(client)

            doctest_namespace["client"] = client
            doctest_namespace["robot"] = robot

            yield
    else:
        yield
