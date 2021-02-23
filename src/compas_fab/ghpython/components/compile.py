import clr
import glob
import os
from compas_fab.ghpython.components import get_ghpy_filename


def compile(version):
    pythonfiles = glob.glob('*.py')
    not_include = ['__init__.py', 'icons.py', 'install.py', os.path.basename(__file__)]
    pythonfiles = list(set(pythonfiles) - set(not_include))
    filename = get_ghpy_filename(version)
    clr.CompileModules(filename, *pythonfiles)


if __name__ == '__main__':
    compile('HERE_TYPE_VERSION_TO_BE_RELEASED')
