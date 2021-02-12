import clr
import glob
import os
from compas_fab.ghpython.components import filename  # only works if Grasshopper is loaded

pythonfiles = glob.glob("*.py")
not_include = ['__init__.py', 'icons.py', 'install.py', os.path.basename(__file__)]
pythonfiles = list(set(pythonfiles) - set(not_include))

clr.CompileModules(filename, *pythonfiles)
