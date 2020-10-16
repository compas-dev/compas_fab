from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
if compas.RHINO:
    import compas_rhino.install
    import compas_rhino.uninstall

import compas_fab.rhino.install  # noqa: E402

__all__ = []


# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument('-v', '--version', choices=['5.0', '6.0'], default='5.0', help="The version of Rhino to install the packages in.")

    args = parser.parse_args()

    packages = set(compas_rhino.install.INSTALLABLE_PACKAGES + compas_fab.rhino.install.INSTALLABLE_PACKAGES)

    compas_rhino.uninstall.uninstall(version=args.version, packages=packages)
