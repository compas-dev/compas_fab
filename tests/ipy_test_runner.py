from __future__ import print_function

import os

import pytest

HERE = os.path.dirname(__file__)


if __name__ == "__main__":
    # Fake Rhino modules
    pytest.load_fake_module("Rhino")
    pytest.load_fake_module("Rhino.Geometry", fake_types=["RTree", "Sphere", "Point3d"])

    # Exclude PyBullet tests in Iron Python environment
    exclude_list = [
        "tests/backends/pybullet/test_pybullet_client.py",
        "tests/backends/pybullet/test_pybullet_planner.py",
    ]

    pytest.run(HERE, exclude_list=exclude_list)
