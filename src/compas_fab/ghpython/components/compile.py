from shutil import copyfile
import os
import clr

from .assembly_info import assembly_name
from .assembly_info import assembly_version

filename = "%s_%s.ghpy" % (assembly_name, assembly_version)
clr.CompileModules(filename,
                   "assembly_info.py",
                   "ros_connect.py",
                   "ros_robot.py",
                   "planning_scene.py",
                   "collision_mesh.py")

src = os.path.join(os.path.dirname(__file__), filename)
dst = os.path.join(r'C:\Users\rustr\AppData\Roaming\Grasshopper\Libraries', filename)
copyfile(src, dst)
