from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta
from abc import abstractmethod


class ForwardKinematics(object):
    __metaclass__ = ABCMeta

    def __call__(self, robot, configuration, group=None, options={}):
        return self.forward_kinematics(robot, configuration, group, options)

    @abstractmethod
    def forward_kinematics(self, robot, configuration, group=None, options={}):
        pass


class InverseKinematics(object):
    __metaclass__ = ABCMeta

    def __call__(self, frame_WCF, start_configuration=None, group=None, options={}):
        return self.inverse_kinematics(frame_WCF, start_configuration, group, options)

    @abstractmethod
    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options={}):
        pass


class PlanMotion(object):
    __metaclass__ = ABCMeta

    def __call__(self, goal_constraints, start_configuration=None, group=None, options={}):
        return self.plan_motion(goal_constraints, start_configuration, group, options)

    @abstractmethod
    def plan_motion(self, goal_constraints, start_configuration=None, group=None, options={}):  # !!!
        pass


class PlanCartesianMotion(object):
    __metaclass__ = ABCMeta

    def __call__(self, frames_WCF, start_configuration=None, group=None, options={}):
        return self.plan_cartesian_motion(frames_WCF, start_configuration, group, options)

    @abstractmethod
    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, group=None, options={}):  # !!!
        pass


class PlanningScene(object):
    __metaclass__ = ABCMeta

    def __call__(self):
        return self.get_planning_scene()

    @abstractmethod
    def get_planning_scene(self):
        pass
