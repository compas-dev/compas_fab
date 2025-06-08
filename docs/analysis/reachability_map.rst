.. _reachability_map:

********************************************************************************
Reachability Map
********************************************************************************

For workspace planning or robot path planning, it is useful to calculate and
visualize the space of the robot's reachability, depending on its attached tool,
obstacles in the environment and its own kinematic and geometric constraints.

The ``ReachabilityMap`` is a collection of all valid IK solutions, i.e. poses
that the end-effector can reach, at specified locations. The map is built by
discretizing the robot's environment, defining frames to be checked,
and calculating the IK solutions for each frame.

The creation of this map depends on the availability of an analytical inverse
kinematic solver for the used robot. Please checkout the kinematic backend for
available robots.

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :glob:

    reachability_map/*

Links
=====
* `Reuleaux (ROS's reachability map) <http://wiki.ros.org/reuleaux>`_
