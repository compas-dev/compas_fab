"""Shared failure diagnostics for the planning Grasshopper components."""

from compas_fab.backends.exceptions import BackendFeatureNotSupportedError
from compas_fab.backends.exceptions import CollisionCheckError


def _name(obj):
    """The `.name` of a collision-pair member, falling back to its string form."""
    return getattr(obj, "name", None) or str(obj)


def collision_diagnostic(planner, state, group=None):
    """A concise one-line hint about why planning/IK failed, via a collision check.

    Runs ``planner.check_collision(state)`` and summarises the result:

    * in collision -> ``"start state in collision: a<->b, c<->d, +N more"``
    * collision-free -> ``"start state collision-free (likely unreachable)"``
    * planner has no collision check -> ``None`` (caller reports the bare error)

    Never raises: a diagnostic must not mask the real failure.

    Parameters
    ----------
    planner
        The planner to query (any backend).
    state
        The robot cell state to check (typically the planning `start_state`).
    group
        Optional planning group to check. Defaults to the planner's main group.

    Returns
    -------
    str or None
    """
    options = {"group": group} if group else None
    try:
        planner.check_collision(state, options=options)
    except CollisionCheckError as cc:
        pairs = getattr(cc, "collision_pairs", None) or []
        listed = ", ".join("{}<->{}".format(_name(a), _name(b)) for a, b in pairs[:3])
        if len(pairs) > 3:
            listed += ", +{} more".format(len(pairs) - 3)
        return "start state in collision: {}".format(listed) if listed else "start state in collision"
    except (BackendFeatureNotSupportedError, AttributeError, NotImplementedError):
        return None
    except Exception:
        return None
    return "start state collision-free (likely unreachable)"
