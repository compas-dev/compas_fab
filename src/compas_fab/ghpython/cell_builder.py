"""Helper for registering tools and rigid bodies into a RobotCell.

Used by the `Load Robot Cell From …` Grasshopper components so that tools and
rigid bodies are registered as part of loading the cell, rather than chained on
afterwards with separate `Add* → set_robot_cell()` steps (which is error-prone:
forgetting the `set_robot_cell()` push makes planning fail on a cell/planner
mismatch).

This only registers the *models* into `cell.tool_models` / `cell.rigid_body_models`.
Attachment (tool→group, body→link/tool) lives in the `RobotCellState` and stays in
the `Attach*` components.
"""

from copy import deepcopy
from typing import Optional

from compas_robots import ToolModel

from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCell


def _explicit_name(item):
    """Return the explicitly-set name of a compas Data item, or '' if unset.

    ``item.name`` falls back to the class name when no name was set (a
    ``compas.data.Data`` behaviour), which is useless as a registration key.
    The ``_name`` backing field is ``None`` until a name is explicitly assigned,
    so it is the reliable "was a name set?" signal.
    """
    return (getattr(item, "_name", None) or "").strip()


def register_models_into_cell(
    component,
    robot_cell: RobotCell,
    tools: Optional[list[ToolModel]] = None,
    rigid_bodies: Optional[list[RigidBody]] = None,
) -> Optional[RobotCell]:
    """Return a deepcopy of ``robot_cell`` with the given tools and rigid bodies registered.

    Tools are keyed by ``tool.name``; rigid bodies by ``body.name``. The input cell is
    never mutated, so a sticky-cached base cell can be passed safely. Missing or duplicate
    ids are reported via :func:`compas_ghpython.error` on ``component`` and skipped.

    Parameters
    ----------
    component
        The Grasshopper component (``ghenv.Component``) used to surface errors.
    robot_cell
        The base cell to copy and extend.
    tools
        Tools to register. Each must have a non-empty ``.name``.
    rigid_bodies
        Rigid bodies to register. Each must have a non-empty ``.name``.

    Returns
    -------
    :class:`compas_fab.robots.RobotCell`
        A new cell with the models registered. If ``robot_cell`` is ``None``, returns ``None``.
    """
    from compas_ghpython import error  # imported lazily; only available in the GH/Rhino runtime

    if robot_cell is None:
        return None

    cell = deepcopy(robot_cell)

    for tool in tools or []:
        if tool is None:
            continue
        tool_id = _explicit_name(tool)
        if not tool_id:
            error(component, "A tool has no name; set one on the Tool From… component so it can be registered.")
            continue
        if tool_id in cell.tool_models:
            error(component, "Duplicate tool id '{}'; a tool with that name is already in the cell. Skipping.".format(tool_id))
            continue
        cell.tool_models[tool_id] = tool

    for body in rigid_bodies or []:
        if body is None:
            continue
        body_id = _explicit_name(body)
        if not body_id:
            error(component, "A rigid body has no name; set one on the Rigid Body From… component so it can be registered.")
            continue
        if body_id in cell.rigid_body_models:
            error(component, "Duplicate rigid body id '{}'; a body with that name is already in the cell. Skipping.".format(body_id))
            continue
        cell.rigid_body_models[body_id] = body

    return cell
