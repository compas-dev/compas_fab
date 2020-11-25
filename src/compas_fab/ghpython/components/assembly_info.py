import GhPython
import System

assembly_name = "COMPAS FAB"
assembly_version = "0.0.1"


class AssemblyInfo(GhPython.Assemblies.PythonAssemblyInfo):
    def get_AssemblyName(self):
        return assembly_name

    def get_AssemblyDescription(self):
        return """Components for the COMPAS FAB package."""

    def get_AssemblyVersion(self):
        return assembly_version

    def get_AuthorName(self):
        return "rust@arch.ethz.ch"

    def get_Id(self):
        return System.Guid("237883f1-9678-4abe-9e20-2ceed9e6c611")
