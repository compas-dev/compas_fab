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
* mm (scale model) to m?
* degrees (for Config) or radians?

# [Draft] Schedule pre-workshop
* 09:00
    * Brief intro to COMPAS (Tom will provide some material)
    * Overview about `compas_fab` (GC)
        * Tools for planning and execution of robotic fabrication
        * CAD-Agnostic modeling of robotic fabrication process: `Frame`, `Transformation`, `Robot`, `Link`, `Joint`, etc
        * Multiple backends for simulation and execution: V-REP Simulator, ROS Client.
    * Path planning
        * Rationale (why is it important in architecture, etc) (AG)
        * Implementation (two implementations: V-REP and MoveIt) (GC)
* 10:30 Break 15’
* 10:45
    * ROS concepts:
      * `Master & Nodes`
      * `Messages, Topics, Services (blocking) & Actions (non-blocking)`
      * `Message and Service Types`
      * `URDF`
      * `Transform library: tf2`
      * `Parameter Server`, `ROS Units` [(link)](http://www.ros.org/reps/rep-0103.html)
      * `RViz`, `rqt_graph`, `roswtf`
      * `ROS Distributions`
    * Robot control with compas_fab + ROS
* 12:00 Lunch break 60’
* 13:00
    * Hands-on session:
      * Setup: install `compas` & `compas_fab` using conda recipe
      * Exercise: examples with `Frame` & `Transformation`
      * Exercise: load robot model and show same model in both Rhino and Blender
      * Setup: start up V-REP either as dockerized app or from the Raspberry Pi
      * Exercise: Examples with V-REP path planner:
        * Test connection
        * Simple forward kinematics
        * Simple inverse kinematics
        * Basic path planning
        * Advanced path planning
      * Setup: connect Raspberry Pi to use ROS (mention WSL?)
      * Exercise: Examples with ROS:
        * Test connection to ROS
        * Load robot model from ROS
        * Inverse kinematics over MoveIt
        * Path planning over MoveIt (UR, from A to B)
      * Demo (instructors): control UR (real) sending path and wait until done, then send again.
      * Demo (instructors): control UR (real) sending path and turning IOs on/off while moving.
      * Demo (instructors): control ABB (simulation) (TBD)
