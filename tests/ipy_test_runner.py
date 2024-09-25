from __future__ import print_function

import os

import pytest

HERE = os.path.dirname(__file__)


if __name__ == "__main__":
    # Fake Rhino modules
    pytest.load_fake_module("Rhino")
    pytest.load_fake_module("Rhino.Geometry", fake_types=["RTree", "Sphere", "Point3d"])

    # Exclude PyBullet tests in Iron Python environment
    # NOTE: 'exclude_list' does not accept wildcards, so we need to find all the files in
    #       the directory "tests/backends/pybullet/"
    pybullet_filenames = os.listdir(os.path.join(HERE, "backends", "pybullet"))
    pybullet_filenames = [
        "tests/backends/pybullet/" + filename for filename in pybullet_filenames if filename.endswith(".py")
    ]
    print(pybullet_filenames)

    exclude_list = pybullet_filenames

    pytest.run(HERE, exclude_list=exclude_list)
