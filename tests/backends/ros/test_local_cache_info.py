from __future__ import print_function

import os

from compas_fab.backends.ros.client import LocalCacheInfo


def test_local_cache_from_none():
    info = LocalCacheInfo.from_local_cache_directory(None)
    assert info.use_local_cache is False


def test_directory_extraction_works_cross_platform():
    local_directory = os.path.join(os.path.expanduser("~"), "robot_description", "robocop")
    info = LocalCacheInfo.from_local_cache_directory(local_directory)
    assert info.use_local_cache is True
    assert info.robot_name == "robocop"
    assert info.local_cache_directory == os.path.join(os.path.expanduser("~"), "robot_description")
