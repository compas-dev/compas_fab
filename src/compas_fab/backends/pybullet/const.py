from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from collections import namedtuple

from compas_fab.utilities import LazyLoader

pybullet = LazyLoader('pybullet', globals(), 'pybullet')

CONNECTION_TYPE = {
    'direct': pybullet.DIRECT,
    'gui': pybullet.GUI,
    'shared_memory': pybullet.SHARED_MEMORY,
    'udp': pybullet.UDP,
    'tcp': pybullet.TCP,
    'gui_server': pybullet.GUI_SERVER,
    'shared_memory_server': pybullet.SHARED_MEMORY_SERVER,
    'shared_memory_gui': pybullet.SHARED_MEMORY_GUI,
}

BASE_LINK_ID = -1
NULL_ID = -1

STATIC_MASS = 0

GREY = (0.5, 0.5, 0.5, 1)
RED = (1, 0, 0, 1)

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
ConstraintInfo = namedtuple('ConstraintInfo', ['constraint_id', 'body_id', 'robot_uid'])
JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])
LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])
