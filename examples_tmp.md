# Examples for compas_fab introduction

1. Introduction of `Frame`, `Transformation`, (RR)
   * Better examples (RR)
1. Introduction of `Pose`, `BaseConfiguration`, `Link`, `Joint`, `Robot`, `UR`, `ABB`
   * Check overlaps of ROS `Pose` and other classes
   * Clean `Robot`, `Link`, `Joint`, `Tool`
   * Change `BaseConfiguration`
1. Robot model and kinematic chain (have the same model in Rhino and Blender?)
   * Robot model from URDF (GC)
1. Plan a path with XX robot in setup (change robot model, calculate new path, obstacles?)
   * For ROBArch (Pre-)WS: With Vrep? (AG)
   * For ROBArch Pre-WS: With moveit? (GC, RR)
1. Robot communication
   * Changing the driver (ABB, UR)?
   * Extend to cartesian pose?
1. Send path to robot and execute. Wait until done to send again.
1. Send path to robot and turn digital output on/off while moving.

TODO:
* rename grasshopper to ghpython
* ROS in a box

# [Draft] Schedule pre-workshop
* 09:00
    * Brief intro to COMPAS (perhaps Tom is interested in presenting this part, I have asked him in a separate email)
    * Overview about compas_fab (GC)
    * Path planning
        * Rationale (AG)
        * Implementation (GC)
* 10:30 Break 15’
* 10:45
    * ROS concepts
    * Robot control with compas_fab + ROS
* 12:00 Lunch break 60’
* 13:00
    * Hands-on session
    * Setup RobotStudio & libraries on every machine (Send upfront instructions to participants to prepare setup if possible)
    * Run examples on simulation (on each participants’ machine)
    * Demo at the RFL (only on instructors’ machine)
