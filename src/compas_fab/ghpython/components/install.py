import argparse
import glob
import os
import shutil

from compas_ghpython import get_grasshopper_library_path
from compas_ghpython import get_grasshopper_userobjects_path
from compas_rhino import _check_rhino_version
from compas._os import create_symlinks
from compas._os import remove_symlinks


def get_version_from_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--version', choices=['5.0', '6.0', '7.0'], default='6.0')
    args = parser.parse_args()
    return _check_rhino_version(args.version)


def install():
    """Installs the Grasshopper components library."""
    results = []

    try:
        version = get_version_from_args()
        grasshopper_library_path = get_grasshopper_library_path(version)

        for f in _remove_old_ghpy_components(grasshopper_library_path):
            results.append(('compas_fab', 'Uninstalled existing component: {}'.format(f), True))

        dstdir = get_grasshopper_userobjects_path(version)
        srcdir = os.path.join(os.path.dirname(__file__), 'components')
        userobjects = glob.glob(os.path.join(srcdir, '*.ghuser'))

        symlinks = []
        for src in userobjects:
            dst = os.path.join(dstdir, os.path.basename(src))
            symlinks.append((src, dst))

        create_symlinks(symlinks)

        results.append(('compas_fab', 'Installed {} GH User Objects on {}'.format(len(userobjects), dstdir), True))
    except PermissionError:
        raise Exception('Please close first all instances of Rhino and then rerun the command')

    return results


def uninstall():
    """Uninstalls the Grasshopper components library."""
    results = []

    try:
        version = get_version_from_args()
        grasshopper_library_path = get_grasshopper_library_path(version)

        for f in _remove_old_ghpy_components(grasshopper_library_path):
            results.append(('compas_fab', 'Uninstalled old component: {}'.format(f), True))

        dstdir = get_grasshopper_userobjects_path(version)
        srcdir = os.path.join(os.path.dirname(__file__), 'components')
        userobjects = glob.glob(os.path.join(srcdir, '*.ghuser'))

        symlinks = []
        for src in userobjects:
            dst = os.path.join(dstdir, os.path.basename(src))
            symlinks.append((src, dst))

        remove_symlinks(symlinks)

        results.append(('compas_fab', 'Uninstalled {} GH User Objects'.format(len(userobjects)), True))
    except PermissionError:
        raise Exception('Please close first all instances of Rhino and then rerun the command')

    return results


def _remove_old_ghpy_components(grasshopper_library_path):
    oldlibs = glob.glob('%s*.ghpy' % os.path.join(grasshopper_library_path, 'COMPAS FAB 0.1'))
    for f in oldlibs:
        os.remove(f)
        yield f


if __name__ == '__main__':
    install()
