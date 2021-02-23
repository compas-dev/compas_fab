import argparse
import glob
import os
import shutil

from compas_ghpython import get_grasshopper_library_path
from compas_rhino import _check_rhino_version

from compas_fab.ghpython.components import assembly_name
from compas_fab.ghpython.components import get_ghpy_filename


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

        for f in _remove_old_components(grasshopper_library_path):
            results.append(('compas_fab', 'Uninstalled existing component: {}'.format(f), True))

        src = os.path.join(os.path.dirname(__file__), get_ghpy_filename())
        dst = os.path.join(grasshopper_library_path, get_ghpy_filename())

        shutil.copyfile(src, dst)

        results.append(('compas_fab', 'GH Components installed on {}'.format(grasshopper_library_path), True))
    except PermissionError:
        raise Exception('Please close first all instances of Rhino and then rerun the command')

    return results


def uninstall():
    """Uninstalls the Grasshopper components library."""
    results = []

    try:
        version = get_version_from_args()
        grasshopper_library_path = get_grasshopper_library_path(version)

        for f in _remove_old_components(grasshopper_library_path):
            results.append(('compas_fab', 'Uninstalled existing component: {}'.format(f), True))
    except PermissionError:
        raise Exception('Please close first all instances of Rhino and then rerun the command')

    return results


def _remove_old_components(grasshopper_library_path):
    oldlibs = glob.glob('%s*.ghpy' % os.path.join(grasshopper_library_path, assembly_name))
    for f in oldlibs:
        os.remove(f)
        yield f


if __name__ == '__main__':
    install()
