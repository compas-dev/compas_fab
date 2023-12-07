"""
********************************************************************************
compas_fab.planning
********************************************************************************

.. currentmodule:: compas_fab.planning

This package contains data classes for modeling robotic process and algorithms for planning robotic motions.
The current implementation supports single-robot (:class:`compas_fab.robots.Robot`)
processes with one or more workpieces (:class:`Workpiece`) and tools (:class:`compas_fab.robots.Tool`).
The processes contains actions that are assumed to be sequentially executed by the robot
or by the operator (manually). Concurrent actions are not supported.

The FabricationProcess class and its subclasses (such as :class:`PickAndPlaceProcess` and
:class:`ExtrusionProcess`) are used to model a process. They provide helper methods for
creating a ordered list of actions that are used for planning and execution. The beining of the
process is defined by the initial state of the scene, which is a container for the state of all
objects in the scene (see :class:`SceneState`). The initial state is used to plan the first action
in the process. The resulting state of the first action is used as the initial state for the next
action, and so on. See tutorial on :ref:`planning_process` for more details.


Actions
--------

Action classes are abstractions of the robot's (and, or the operator's) capability to manipulate tools and
workpieces in the scene. Action classes are used for modeling a process for the following purpose:

* To plan trajectories for robotic motions
* To simulate and visualize the process in a virtual environment
* To execute the process on a real robot or a human-robot collaboration process

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Action
    RoboticAction
    LinearMotion
    FreeMotion
    OpenGripper
    CloseGripper
    ManualWorkpieceMotion

States
------

State classes are used to model the immutable, static state of objects in the planning scene. These include:
:class:`RobotState` for :class:`compas_fab.robots.Robot`,
:class:`ToolState` for :class:`compas_fab.robots.Tool` and
:class:`WorkpieceState` for :class:`compas_fab.robots.Workpiece`.
The :class:`SceneState` class is a container that holds the state of all objects in the scene.

Typically a robotic process will have an initial (starting) state that is defined manually.
If sequential planning is used, the initial state is used to plan the first action in the process.
The resulting state of the first action is used as the initial state for the next action, and so on.
If non-sequential planning is used, the starting state of actions are inferred from the list of
actions in the process. See tutorial on :ref:`planning_process` for more details.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    SceneState
    WorkpieceState
    ToolState
    RobotState


"""

from .action import (
    Action,
    RoboticAction,
    LinearMotion,
    FreeMotion,
    OpenGripper,
    CloseGripper,
    ManuallyMoveWorkpiece,
    HideWorkpieces,
    ShowWorkpieces,
)

from .state import (
    SceneState,
    WorkpieceState,
    ToolState,
    RobotState,
)

__all__ = [
    "Action",
    "RoboticAction",
    "LinearMotion",
    "FreeMotion",
    "OpenGripper",
    "CloseGripper",
    "ManuallyMoveWorkpiece",
    "HideWorkpieces",
    "ShowWorkpieces",
    "SceneState",
    "WorkpieceState",
    "ToolState",
    "RobotState",
]
