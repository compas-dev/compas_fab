from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from collections import namedtuple

PYBULLET_GUI = 1
PYBULLET_DIRECT = 2
PYBULLET_SHARED_MEMORY = 3
PYBULLET_UDP = 4
PYBULLET_TCP = 5
PYBULLET_GUI_SERVER = 7
PYBULLET_SHARED_MEMORY_SERVER = 9
PYBULLET_SHARED_MEMORY_GUI = 14

CONNECTION_TYPE = {
    'direct': PYBULLET_DIRECT,
    'gui': PYBULLET_GUI,
    'shared_memory': PYBULLET_SHARED_MEMORY,
    'udp': PYBULLET_UDP,
    'tcp': PYBULLET_TCP,
    'gui_server': PYBULLET_GUI_SERVER,
    'shared_memory_server': PYBULLET_SHARED_MEMORY_SERVER,
    'shared_memory_gui': PYBULLET_SHARED_MEMORY_GUI,
}

BASE_LINK_ID = -1
NULL_ID = -1

STATIC_MASS = 0

GREY = (0.5, 0.5, 0.5, 1)
RED = (1, 0, 0, 1)

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
ConstraintInfo = namedtuple('ConstraintInfo', ['constraint_id', 'body_id', 'robot_uid'])
JointInfo = namedtuple(
    'JointInfo',
    [
        'jointIndex',
        'jointName',
        'jointType',
        'qIndex',
        'uIndex',
        'flags',
        'jointDamping',
        'jointFriction',
        'jointLowerLimit',
        'jointUpperLimit',
        'jointMaxForce',
        'jointMaxVelocity',
        'linkName',
        'jointAxis',
        'parentFramePos',
        'parentFrameOrn',
        'parentIndex',
    ],
)
JointState = namedtuple(
    'JointState', ['jointPosition', 'jointVelocity', 'jointReactionForces', 'appliedJointMotorTorque']
)
LinkState = namedtuple(
    'LinkState',
    [
        'linkWorldPosition',
        'linkWorldOrientation',
        'localInertialFramePosition',
        'localInertialFrameOrientation',
        'worldLinkFramePosition',
        'worldLinkFrameOrientation',
    ],
)
