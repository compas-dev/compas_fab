"""This module is mostly for explanatory purposes, to have a usable setup
to test and show features onto.

It needs to be explicitely imported, since it is not a fundamental part
of the compas_fab.robots package at all.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.robots import Joint
from compas.robots import Limit
from compas.robots import Link

import compas_fab.robots.robot

ROBOTS = {
    'A': dict(id=11, index=0, base_external_axes=[7., -2., -4.]),
    'B': dict(id=12, index=1, base_external_axes=[7., -10., -4.]),
    'C': dict(id=21, index=2, base_external_axes=[30., -2., -4.]),
    'D': dict(id=22, index=3, base_external_axes=[30., -10., -4.]),
}

__all__ = [
    'Robot',
]


def Robot(name, client=None):
    """Retrieves one of the robots of the setup.

    This method serves mainly as help for tutorials and exercises where
    so that there is a full scene available to play with, instead of having
    to start from zero.

    This function is intentionally capitalized to act as an almost drop-in
    replacement for the constructor of :class:`compas_fab.robots.Robot`.

    Parameters
    ----------
    name : str
        Robot's name.
    client : object
        Backend client.

    Returns
    -------
    :class:`compas_fab.robots.Robot`
        Newly created instance of the requested robot.

    Examples
    --------

    >>> from compas_fab.robots import rfl
    >>> robot = rfl.Robot('A')
    >>> robot.name
    'A'
    """
    attributes = ROBOTS[name]
    joints, links = _create_model()
    robot = compas_fab.robots.robot.Robot.basic(name, joints, links, [], **attributes)

    if client:
        robot.client = client

    return robot


def _create_model():
    joints = [
        Joint('joint_x', 'prismatic', parent='rfl', child='bridge', limit=Limit()),
        Joint('joint_y', 'prismatic', parent='bridge', child='vertical', limit=Limit()),
        Joint('joint_z', 'prismatic', parent='vertical', child='base_link', limit=Limit()),
        Joint('joint_1', 'revolute', parent='base_link', child='link_1', limit=Limit()),
        Joint('joint_2', 'revolute', parent='link_1', child='link_2', limit=Limit()),
        Joint('joint_3', 'revolute', parent='link_2', child='link_3', limit=Limit()),
        Joint('joint_4', 'revolute', parent='link_3', child='link_4', limit=Limit()),
        Joint('joint_5', 'revolute', parent='link_4', child='link_5', limit=Limit()),
        Joint('joint_6', 'revolute', parent='link_5', child='link_6', limit=Limit()),
    ]
    links = [
        Link('rfl'), Link('bridge'), Link('vertical'), Link('base_link'),
        Link('link_1'), Link('link_2'), Link('link_3'), Link('link_4'), Link('link_5'), Link('link_6'),
    ]
    return joints, links
