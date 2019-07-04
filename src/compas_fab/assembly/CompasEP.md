- CompasEP: fab0
- Title: Assembly for robotic assembly data structures
- Author: Yijiang Huang
- Status: Draft
- Type: Informational(?)
- Content-Type: text/x-rst
- Created: 25-Jan-2019
- Post-History: ?

Outline
=======

Abstract
========

This CompasEP proposes an unified data structure for encoding all the necessary
information of an assembly of discrete geometries in an architectural assembly
planning context. The proposed data structure includes information on unit geometries, connectivity, structural modeling, robotic assembly/fabrication. The implemented script is intended as an addition of the `compas_fab` package.

Motivation
==========

**Unit Geometry**: Geometry of unit element

**Connectivity**: Modeling relationship elements
  - Who is connected to who?
  - How are they connected? (contact, extra physical joints)
  - How to describe the solid geometric relationship between the joint and the element? (boolean xxx?)
  - What’s the structural model for this connection? (elastic, contact)
  - Modeling inherent assembly direction (robot-agnostic)? (interlocking, press-fit joint)

**Robotic assembly**: Modeling relationship between robot’s end effector (EE) and unit elements
  - What type of interaction? Prehensile: grasp; Non-prehensile: push, pull, extrude (solder, sew, polish)
  - Candidate EE poses under the element object frame? (Auto-computed? User-provided?)

Definitions
===========

Main
====

A list of element geometries (the `UnitGeometry` can be "folded" and "unfold" to
expose the full geometry upon request)

A list of physical joint geometries (if any)

A `Network`:

**Element vertex**
- element geometry (`UnitGeometry`)
- `get_neighbored_elements`
- `get_neighbored_virtual_joints`
- type of robot EE interaction: grasp, extrude, push, pull, etc.
- Candidate EE poses under the element object frame
(let's do everything discrete now, we can ask user to provide a black-box sampler
to operate on continuous domain as well)

**Virtual joint vertex**
1. Abstract connectivity info
(who am I connecting to, just by indices?)

  -> E1, E4, E6

2. Is there an extra physical joint?

  `physical_joint` attribute, optional (dict):
    - geometry of the joint (if any), `UnitGeometry`
    - pose of the geometry
    - solid geometric relationship between the joint and connected elements
      - Boolean difference/intersection/...
      - Constructive Solid Geometry (CSG) sequence
    - robot grasp poses (w.r.t physical_joint's frame)

3. What is the physical model for this connection?

  - elastic
    - *only support linear elastic analysis on frames (straight central axis) for now*
    - should offer `extract_structural_line_model` function
    - the nodal indices in the structural model shouldn't be emphasized. Always
    query nodal deformation by `element_id` -> `end_pts` -> `end_pt_deformation`
    - if there is no phyiscal joint, find closest point between the central axes,
    and add a frame segment connecting them.

  - contact
    - See `compas_rbe` and `compas_assembly`, TODO

5. Directional block graph (`Network`)
  - Encode the assembly direction info
  - This imposes a partial order on the assembly sequence planning

**unit_geometry**

  Can constructed from parametric shapes (e.g. h, l, w of a beam), static shape (mesh, brep). This could simply be a wrapper offering functionalities on extracting geometric
  entries or properties (collision meshes, central axis, centroid, structural representation info, etc).

  Both `Element` and `VirtualJoint`'s geometry attribute should be of this type.

  - extract ...
  - inherent object frame (default world_XY with origin moved to the centroid)

Todo's
======

Generate assembly_instance (json, URDF, objs) for StripStream
-------------------------------------------------------------


Create network from a list input shapes
---------------------------------------

Using the host CAD softawre's boolean or use/implement a SDF to detect if two assembly geometries are adjacent, and create a network upon this. And we can build AssemblyNetwork upon this.
TODO: This might not make sense - The boolean detection is widely known to be unstable.

induced subgraph on the compas.datastructures.Network
-----------------------------------------------------

Recommendation
--------------
The word `hashable` used in network's doc is a bit confusing for newcomers. Maybe explain it a bit, e.g. [#hashable]_.

Notes
=====
Since this seems to be the first CompasEP, I am following the REP (ROS enhancement proposal) guideline [#REP-0012]_. See [#REP-I0007]_ for a good example.

Dev notes
---------
This contains some notes during the code development.

Default local frame on the centroid?

References
==========

.. [#REP-0012] REP-0012 provides guidelines and templates for REP.
   (http://www.ros.org/reps/rep-0012.html)

.. [#REP-I0007] REP-I0007 gives a good example for writing a proposal.
   (https://github.com/ros-industrial/rep/blob/master/rep-I0007.rs://github.com/ros-industrial/rep/blob/master/rep-I0007.rst)

.. [#hashable] hashable's definition in python.
   (https://stackoverflow.com/questions/14535730/what-do-you-mean-by-hashable-in-python)
