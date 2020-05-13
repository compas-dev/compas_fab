from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta
from abc import abstractmethod


class ForwardKinematics(object):
    __metaclass__ = ABCMeta

    def __call__(self, configuration, group=None, options={}):
        return self.forward_kinematics(configuration, group, options)

    @abstractmethod
    def forward_kinematics(self, configuration, group=None, options={}):
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
    def plan_motion(self, goal_constraints, start_configuration=None, group=None, options={}):
        pass


class PlanCartesianMotion(object):
    __metaclass__ = ABCMeta

    def __call__(self, frames_WCF, start_configuration=None, group=None, options={}):
        return self.plan_cartesian_motion(frames_WCF, start_configuration, group, options)

    @abstractmethod
    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, group=None, options={}):
        pass


class GetPlanningScene(object):
    __metaclass__ = ABCMeta

    def __call__(self, options={}):
        return self.get_planning_scene(options)

    @abstractmethod
    def get_planning_scene(self, options={}):
        pass


class AddCollisionMesh(object):
    __metaclass__ = ABCMeta

    def __call__(self, collision_mesh, options={}):
        return self.add_collision_mesh(collision_mesh, options)

    @abstractmethod
    def add_collision_mesh(self, collision_mesh, options={}):
        pass


class RemoveCollisionMesh(object):
    __metaclass__ = ABCMeta

    def __call__(self, id, options={}):
        return self.remove_collision_mesh(id, options)

    @abstractmethod
    def remove_collision_mesh(self, id, options={}):
        pass


class AppendCollisionMesh(object):
    __metaclass__ = ABCMeta

    def __call__(self, collision_mesh, options={}):
        return self.append_collision_mesh(collision_mesh, options)

    @abstractmethod
    def append_collision_mesh(self, collision_mesh, options={}):
        pass


class AddAttachedCollisionMesh(object):
    __metaclass__ = ABCMeta

    def __call__(self, attached_collision_mesh, options={}):
        return self.add_attached_collision_mesh(attached_collision_mesh, options)

    @abstractmethod
    def add_attached_collision_mesh(self, attached_collision_mesh, options={}):
        pass


class RemoveAttachedCollisionMesh(object):
    __metaclass__ = ABCMeta

    def __call__(self, id, options={}):
        return self.remove_attached_collision_mesh(id, options)

    @abstractmethod
    def remove_attached_collision_mesh(self, id, options={}):
        pass
