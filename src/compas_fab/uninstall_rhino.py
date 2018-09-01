from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import os
import sys

import compas_rhino
import compas_rhino.uninstall
import compas_fab.install_rhino

from compas._os import remove_symlink

__all__ = []


def uninstall(version='5.0', packages=None):
    """Uninstall compas_fab from Rhino.

    Parameters
    ----------
    version : {'5.0', '6.0'}
        The version number of Rhino.
    packages : list of str
        List of packages to uninstall or None to use default package list.

    Examples
    --------
    .. code-block:: python

        >>> import compas_fab
        >>> compas_fab.uninstall_rhino('5.0')

    .. code-block:: python

        $ python -m compas_fab.uninstall_rhino 5.0

    """

    compas_rhino.uninstall.uninstall(version, packages)


# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":

    import sys

    print('\nusage: python -m compas_rhino.uninstall [version]\n')
    print('  version       Rhino version (5.0 or 6.0)\n')

    try:
        version = sys.argv[1]
    except IndexError:
        version = '5.0'
    else:
        try:
            version = str(version)
        except Exception:
            version = '5.0'

    packages = set(compas_rhino.install.INSTALLABLE_PACKAGES +
                   compas_fab.install_rhino.INSTALLABLE_PACKAGES)
    uninstall(version=version, packages=packages)
