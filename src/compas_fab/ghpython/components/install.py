import os
import glob
from shutil import copyfile
import compas

from compas_fab.ghpython.components import assembly_name
from compas_fab.ghpython.components import filename
from compas_rhino import _check_rhino_version

# TODO: function below must move into compas_ghpython


def _get_grasshopper_library_path(version):
    if compas.WINDOWS:
        grasshopper_library_path = os.path.join(os.getenv('APPDATA'), 'Grasshopper', 'Libraries')
    elif compas.OSX:
        grasshopper_library_path = os.path.join(os.getenv('HOME'), 'Library', 'Application Support', 'McNeel', 'Rhinoceros', '{}'.format(version),
                                                'Plug-ins', 'Grasshopper (b45a29b1-4343-4035-989e-044e8580d9cf)', 'Libraries')
    else:
        raise Exception('Unsupported platform')
    return grasshopper_library_path


def install(version=None):
    """Installs the Grasshopper components library.
    """
    try:
        version = _check_rhino_version(version)
        grasshopper_library_path = _get_grasshopper_library_path(version)
        # remove old libraries
        oldlibs = glob.glob("%s*.ghpy" % os.path.join(grasshopper_library_path, assembly_name))
        [os.remove(f) for f in oldlibs]
        src = os.path.join(os.path.dirname(__file__), filename)
        dst = os.path.join(grasshopper_library_path, filename)
        copyfile(src, dst)
    except PermissionError:
        raise Exception("Please close first all instances of Rhino and then rerun the command")


if __name__ == "__main__":
    install()
