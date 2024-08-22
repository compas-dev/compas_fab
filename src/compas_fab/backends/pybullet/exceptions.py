from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class PyBulletError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.pybullet``."""

    def __init__(self, message):
        super(PyBulletError, self).__init__(message)


class PlanningGroupNotSupported(PyBulletError):
    """Exception raised when a problem is caused by a planning group.

    Attributes
    ----------
    group_name : str
        The name of the planning group.
    joint_names : list of str
        The names of the joints in the planning group.
    link_names : list of str
        The names of the links in the planning group.

    """

    def __init__(self, group_name, joint_names, link_names):
        super(PlanningGroupNotSupported, self).__init__(
            "Planning group '{}' not supported by PyBullet".format(group_name)
        )
        self.group_name = group_name
        self.joint_names = joint_names
        self.link_names = link_names
