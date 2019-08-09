"""
*******************************************************************************
compas_fab.backends.pybullet
*******************************************************************************

.. module:: compas_fab.backends.ros

Package with functionality to interact with `pybullet <https://pybullet.org/wordpress//>`_.

Note: this backend is not fully support yet, only utility funcions are provided
to facilitate the integration between compas_fab and the
`pychoreo planner <https://github.com/yijiangh/pychoreo>`. And now it has some
unnecessary convenience functions from `ss-pybullet <https://github.com/caelan/ss-pybullet>`,
which is wrapped under the `conrob_pybullet` module that is shipped with pychoreo.

Warning: we might only support py3.2 because of the use of tempfile.TemporaryDirectory.
But I haven't looked into that too much.

.. autosummary::
    :toctree: generated/



"""

from __future__ import absolute_import

from .interface_utils import *

__all__ = [name for name in dir() if not name.startswith('_')]
