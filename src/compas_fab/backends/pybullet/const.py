from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from collections import namedtuple

from compas.geometry import Frame

BASE_LINK_ID = -1
NULL_ID = -1

ZERO_FRAME = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])

STATIC_MASS = 0

GREY = (0.5, 0.5, 0.5, 1)
RED = (1, 0, 0, 1)  # !!! more colors?

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
ConstraintInfo = namedtuple('ConstraintInfo', ['constraint_id', 'body_id'])
JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])
LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])
