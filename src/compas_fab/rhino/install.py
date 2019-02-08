from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas_rhino.install

__all__ = []

INSTALLABLE_PACKAGES = ['compas_fab', 'roslibpy']


# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--version', choices=['5.0', '6.0'], default='6.0', help="The version of Rhino to install the packages in.")

    args = parser.parse_args()

    packages = set(compas_rhino.install.INSTALLABLE_PACKAGES + INSTALLABLE_PACKAGES)

    compas_rhino.install.install(version=args.version, packages=packages)
