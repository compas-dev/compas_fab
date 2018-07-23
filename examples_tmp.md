# Examples for compas_fab introduction

1. Introduction of `Frame`, `Transformation`, (RR)
   * Better examples (RR)
1. Introduction of `Robot`
    * URDF (GC)
    * `BaseConfiguration` => `Configuration`
    * `Link`, `Joint`,  (URDF) (GC)
    * `Robot`,  (RR, KD): convention, make describtion, make example, (look up in simulater rfl)
      * Client: connect to simulator or real robot
    * `UR` (RR): inherits from Robot, inverse (through ROS and direct), forward
    * `ABB` (KD)
   * `Pose`: Frame + dict in RCS
   * `Tool`
   * Example: Show same model in Rhino and Blender
1. Pathplanning lan a path with XX robot in setup (change robot model, calculate new path, obstacles?)
   * For ROBArch Pre-WS: Plan path with RFL (Vrep) (AG)
   * For ROBArch Pre-WS: With moveit (GC, RR)
      load UR, set 2 planes, send it over roslibpy to model movit, get path back.
      (change tool dynamically?, add box?)
1. Robot communication
   * UR in real (send path to UR), ABB in simulation?
   (NOT FOR WORKSHOP:  Extend to cartesian pose)
   * Example 1: Send path to robot and execute. Wait until done to send again.
   * Example 2: Send path to robot and turn digital output on/off while moving.

TODO:
* rename grasshopper to ghpython
* rename compas_fab to compas_rfab or compas_robfab
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
