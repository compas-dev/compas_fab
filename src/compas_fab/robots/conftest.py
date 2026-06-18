import math
import os
import time

import compas
import pytest
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Rotation
from compas.geometry import Scale
from compas.geometry import allclose

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import RobotCellLibrary


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
    doctest_namespace["RobotCellLibrary"] = RobotCellLibrary
    doctest_namespace["RosClient"] = RosClient
    doctest_namespace["Rotation"] = Rotation
