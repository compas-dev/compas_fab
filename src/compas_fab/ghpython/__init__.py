"""Grasshopper (GHPython) scene objects and component helpers for compas_fab.

Scene objects ([`RobotCellObject`][compas_fab.ghpython.RobotCellObject],
[`RigidBodyObject`][compas_fab.ghpython.RigidBodyObject], etc.) expose
`compas_fab` data structures to Grasshopper while keeping the data layer
separated from CAD-specific code and leveraging native Rhino/Grasshopper
performance.

Alongside them are the helpers the `Cf_*` components call to register models into
a cell ([`register_models_into_cell`][compas_fab.ghpython.register_models_into_cell]),
auto-build per-joint sliders ([`ensure_joint_sliders`][compas_fab.ghpython.ensure_joint_sliders]),
cache scene objects across solves ([`cache_scene_object`][compas_fab.ghpython.cache_scene_object]),
and auto-create dropdowns ([`ensure_value_list`][compas_fab.ghpython.ensure_value_list]) or
boolean toggles ([`ensure_boolean_toggle`][compas_fab.ghpython.ensure_boolean_toggle]) on
component inputs.

Value-list dropdowns come in two flavours:

* [`ensure_value_list`][compas_fab.ghpython.ensure_value_list] — fire-and-forget,
  for static option sets (e.g. the fixed list of robots in `RobotCellLibrary`).
  Creates the value list when nothing is wired, then never touches it again.

* [`ensure_dynamic_value_list`][compas_fab.ghpython.ensure_dynamic_value_list] —
  for options that depend on upstream data (e.g. the keys of `cell.tool_models`).
  Tracks the value list it created via `scriptcontext.sticky` and refreshes its
  items whenever the option set changes. Defers the canvas mutation to a
  `ScheduleSolution(delay, callback)` callback so the rebuild happens between
  solves rather than expiring downstream consumers mid-solve.
"""

import compas

if compas.RHINO:
    from .scene import (
        ReachabilityMapObject,
        RigidBodyObject,
        RobotCellObject,
        RobotModelObject,
    )
    from .cell_builder import register_models_into_cell
    from .joint_sliders import ensure_joint_sliders
    from .moveit_options import MoveItPlannerOptions
    from .sticky_cache import cache_scene_object
    from .value_list import ensure_boolean_toggle
    from .value_list import ensure_dynamic_value_list
    from .value_list import ensure_value_list

    __all__ = [
        "MoveItPlannerOptions",
        "ReachabilityMapObject",
        "RigidBodyObject",
        "RobotCellObject",
        "RobotModelObject",
        "cache_scene_object",
        "ensure_boolean_toggle",
        "ensure_dynamic_value_list",
        "ensure_joint_sliders",
        "ensure_value_list",
        "register_models_into_cell",
    ]
