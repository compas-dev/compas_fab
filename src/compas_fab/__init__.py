"""
********************************************************************************
compas_fab
********************************************************************************

.. currentmodule:: compas_fab

This library provides tools to plan and execute robotic fabrication processes using
the COMPAS Framework with a special focus on enabling its usage from within CAD
environments.

.. toctree::
    :maxdepth: 3

    compas_fab.backends
    compas_fab.robots
    compas_fab.utilities
    compas_fab.sensors
    compas_fab.blender
    compas_fab.ghpython
    compas_fab.rhino

"""

import os

from .__version__ import __author__
from .__version__ import __author_email__
from .__version__ import __copyright__
from .__version__ import __description__
from .__version__ import __license__
from .__version__ import __title__
from .__version__ import __url__
from .__version__ import __version__

HERE = os.path.dirname(__file__)
HOME = os.path.abspath(os.path.join(HERE, '../..'))
DATA = os.path.abspath(os.path.join(HERE, 'data'))


def _find_resource(filename):
    filename = filename.strip('/')
    return os.path.abspath(os.path.join(DATA, filename))


def get(filename):
    return _find_resource(filename)


# Check if COMPAS is installed from git
# If that's the case, try to append the current head's hash to __version__
try:
    git_head_file = os.path.abspath(os.path.join(HOME, '.git', 'HEAD'))

    if os.path.exists(git_head_file):
        # git head file contains one line that looks like this:
        # ref: refs/heads/master
        with open(git_head_file, 'r') as git_head:
            _, ref_path = git_head.read().strip().split(' ')
            ref_path = ref_path.split('/')

            git_head_refs_file = os.path.abspath(os.path.join(HOME, '.git', *ref_path))

        if os.path.exists(git_head_refs_file):
            with open(git_head_refs_file, 'r') as git_head_ref:
                git_commit = git_head_ref.read().strip()
                __version__ += '-' + git_commit[:8]
except Exception:
    pass

__all_plugins__ = ['compas_fab.rhino.install']
__all__ = ['__author__', '__author_email__', '__copyright__', '__description__', '__license__', '__title__', '__url__', '__version__', 'get']
