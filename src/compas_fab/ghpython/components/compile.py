import os
from shutil import copyfile

import clr

assembly_name = "COMPAS FAB"
assembly_version = "0.0.1"

filename = "%s_%s.ghpy" % (assembly_name, assembly_version)
clr.CompileModules(filename,
                   "assembly_info.py",
                   "ros_connect.py",
                   "ros_robot.py",
                   "planning_scene.py",
                   "collision_mesh.py",
                   "attached_collision_mesh.py",
                   "inverse_kinematics.py",
                   "plan_cartesian_motion.py",
                   "constraints_from_plane.py",
                   "plan_motion.py",
                   "visualize_robot.py",
                   "visualize_trajectory.py",
                   )


appdata = os.getenv('APPDATA')
os.path.join(appdata, 'Grasshopper', 'Libraries')  # TODO: must move into compas_ghpython

src = os.path.join(os.path.dirname(__file__), filename)
dst = os.path.join(r'C:\Users\rustr\AppData\Roaming\Grasshopper\Libraries', filename)
copyfile(src, dst)
