"""This module is mostly for example purposes.

It needs to be explicitely imported, since it is not a fundamental part
of the compas_fab.robots package at all.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.robots import LocalPackageMeshLoader
from compas.robots import RobotModel

import compas_fab
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics

__all__ = [
    'Robot',
]


def Robot(client=None, load_geometry=False):
    """Returns a UR5 robot.

    This method serves mainly as help for writing examples, so that the code can
    stay short.

    It is intentionally capitalized to act as an almost drop-in replacement for
    the constructor of :class:`compas_fab.robots.Robot`.

    Parameters
    ----------
    client: object
        Backend client.
    load_geometry: bool

    Returns
    -------
    :class:`compas_fab.robots.Robot`
        Newly created instance of a UR5 robot.

    Examples
    --------

    >>> from compas_fab.robots.ur5 import Robot
    >>> robot = Robot()
    >>> robot.name
    'ur5'
    """

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    if load_geometry:
        loader = LocalPackageMeshLoader(compas_fab.get('universal_robot'), 'ur_description')
        model.load_geometry(loader)

    robot = RobotClass(model, semantics=semantics)

    if client:
        robot.client = client

    return robot
