# Universal Robots model assets

The UR3, UR3e, UR5, UR5e, UR10, UR10e, and UR16e meshes and the source data
used to generate their bundled URDF files come from
[`UniversalRobots/Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description),
release `4.3.1`, commit `ae333289875f9ba5a9ea6649a54036efb5ccabee`.

The URDF files were flattened from `urdf/ur.urdf.xacro` with the matching
`config/<robot>/` parameters. The top-level world link from the demonstration
wrapper was omitted so `base_link` remains the root of each local cell, matching
the library's existing API contract. Visual Collada meshes and collision STL
files are unchanged from upstream. The mesh package is shared by all seven local
robot-cell definitions to avoid duplicating identical upstream assets.

These models and meshes are covered by the adjacent `LICENSE` file. The
additional graphical-documentation terms mentioned upstream do not apply to
the seven model families bundled here.
