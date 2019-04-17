*******************************************************************************
Plan motion
*******************************************************************************

There are 2 function that allow to plan a robotic movement without collisions:
`plan_cartesian_motion` and `plan_motion`.

.. More coming soon ...

Plan cartesian motion
=====================

.. literalinclude :: files/04_plan_cartesian_motion.py
   :language: python

Plan motion
===========

In contrast to the cartesian path, the `plan_motion` allows to describe the
goal with constraints rather than defined frames.

.. literalinclude :: files/04_plan_motion.py
   :language: python
