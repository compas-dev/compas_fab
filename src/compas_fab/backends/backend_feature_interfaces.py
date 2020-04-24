from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta
from abc import abstractmethod


class InverseKinematics(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def inverse_kinematics(self, frames_WCF, start_configuration=None, group=None, options=None):
        pass


class ForwardKinematics(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def forward_kinematics(self, robot, configuration, group=None, options=None):
        pass


class PlanMotion(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def plan_motion(self, ):
        pass


class PlanCartesianMotion(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def plan_cartesian_motion(self, ):
        pass
