from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys

import compas_rhino.install

__all__ = []

INSTALLABLE_PACKAGES = ('compas_fab', 'roslibpy')


def install(version='5.0', packages=None):
    """Install compas_fab for Rhino.

    Parameters
    ----------
    version : {'5.0', '6.0'}
        The version number of Rhino.
    packages : list of str
        List of packages to install or None to use default package list.

    Examples
    --------
    .. code-block:: python

        >>> import compas_fab
        >>> compas_fab.rhino.install('5.0')

    .. code-block:: python

        $ python -m compas_fab.rhino.install 5.0

    """

    compas_rhino.install.install(version, packages)


# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":

    import sys

    print('\nusage: python -m compas_fab.rhino.install [version]\n')
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

    packages = set(compas_rhino.install.INSTALLABLE_PACKAGES + INSTALLABLE_PACKAGES)
    install(version=version, packages=packages)
