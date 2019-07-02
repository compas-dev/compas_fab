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

This CompasEP proposes an unified data structure for describing discrete geometries
in robotic assembly project. The implemented script is intended as an addition of the `compas_fab` package.

Motivation
==========

Inspired by the `Beams_Network` used in `timber_workshop_robarch`. Thought that this should be generalized and open for general `compas_fab` users.

Definitions
===========
TODO: port the `what do we need from this data structure?` section from my slides.

Main
====

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

References
==========

.. [#REP-0012] REP-0012 provides guidelines and templates for REP.
   (http://www.ros.org/reps/rep-0012.html)

.. [#REP-I0007] REP-I0007 gives a good example for writing a proposal.
   (https://github.com/ros-industrial/rep/blob/master/rep-I0007.rs://github.com/ros-industrial/rep/blob/master/rep-I0007.rst)

.. [#hashable] hashable's definition in python.
   (https://stackoverflow.com/questions/14535730/what-do-you-mean-by-hashable-in-python)
