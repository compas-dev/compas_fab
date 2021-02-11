import GhPython
import System

from compas_fab.ghpython.components import assembly_name
from compas_fab.ghpython.components import assembly_version
from compas_fab.__version__ import __author_email__

class AssemblyInfo(GhPython.Assemblies.PythonAssemblyInfo):
    def get_AssemblyName(self):
        return assembly_name

    def get_AssemblyDescription(self):
        return """Components for the COMPAS FAB package."""

    def get_AssemblyVersion(self):
        return assembly_version

    def get_AuthorName(self):
        return __author_email__

    def get_Id(self):
        return System.Guid("237883f1-9678-4abe-9e20-2ceed9e6c611")
