.. _plan_motion:

*******************************************************************************
Plan Motion
*******************************************************************************

Motion Planning is the process of finding a trajectory that moves the robot from
an initial state to a goal state. The trajectory is a sequence of configurations
(joint values) that the robot should pass through in order to reach the goal.

Different from performing an Inverse Kinematics (IK) calculation, which finds only
the configuration for a single target, motion planning (MP) finds a sequence of
configurations that are continuous and collision-free.

The motion planning problem is complex and in many cases has many solutions.
The most common approach is to use a sampling-based planner, which generates a
large number of candidate solutions and selects the best one. All of the planning
backends in COMPAS FAB use sampling-based planners for free-motion planning.

One property of sampling-based planners is that they are probabilistic. This means
that they may not always find a solution, or they may find a solution that is not
optimal. The planner may also take a long time to find a solution, or it may find
a solution quickly. The planner may also find different solutions each time it is
run.

The motion planning problem is complex because it involves many constraints and
requirements. The robot must avoid collisions with obstacles, it must respect the
limits of its joints, it must respect the limits of its velocity and acceleration,
and the trajectory must be continuous.


Collision Avoidance
===================

Collision detection between



and it must respect the limits of its end-effector.