import clr

# The below line does not work, since Grasshopper cannot be loaded from withing RhinoPython Editor.
#from compas_fab.ghpython.components import filename

assembly_name = "COMPAS FAB"
assembly_version = "0.15.0"
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
