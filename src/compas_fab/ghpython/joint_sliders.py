"""Helper for auto-creating per-joint Number Sliders wired into one list input.

The hosting component declares a single `joints` input with list access. This
helper auto-creates one `GH_NumberSlider` per configurable joint and wires
them all to that input in canonical joint order. Because the input is part
of the script signature, GH collects its data via the normal solve path and
the script receives `joints` as a plain Python list of floats — no
`VolatileData` dance required.
"""

import math

import compas

if compas.RHINO:
    import Grasshopper
    import System


_DEFAULT_CONTINUOUS_RANGE = 2 * math.pi
_DEFAULT_PRISMATIC_RANGE = 1.0


def _joint_signature(robot_model):
    """Stable identity for the configurable joints — name, type, lower, upper."""
    sig = []
    for joint in robot_model.get_configurable_joints():
        lower = joint.limit.lower if joint.limit else None
        upper = joint.limit.upper if joint.limit else None
        sig.append((joint.name, joint.type, lower, upper))
    return tuple(sig)


def _resolve_limits(joint):
    """Return (lower, upper, default) for a joint, falling back to sensible
    ranges when the URDF doesn't supply limits (continuous joints, etc.)."""
    from compas_robots.model import Joint

    lower = joint.limit.lower if joint.limit else None
    upper = joint.limit.upper if joint.limit else None

    if lower is None or upper is None or lower == upper:
        if joint.type == Joint.PRISMATIC:
            lower = -_DEFAULT_PRISMATIC_RANGE
            upper = _DEFAULT_PRISMATIC_RANGE
        else:
            lower = -_DEFAULT_CONTINUOUS_RANGE
            upper = _DEFAULT_CONTINUOUS_RANGE

    default = max(lower, min(upper, 0.0))
    return lower, upper, default


def _create_slider(joint, lower, upper, default, pivot):
    slider = Grasshopper.Kernel.Special.GH_NumberSlider()
    slider.CreateAttributes()
    # SetInitCode "min<value<max" is the documented shortcut for range+value.
    slider.SetInitCode("{:.6f}<{:.6f}<{:.6f}".format(lower, default, upper))
    slider.NickName = joint.name
    slider.Attributes.Pivot = pivot
    slider.Attributes.ExpireLayout()
    return slider


def _is_number_slider(obj):
    """Robust slider check.

    ``isinstance(obj, Grasshopper.Kernel.Special.GH_NumberSlider)`` is unreliable
    in Rhino 8 CPython because of .NET interface interop quirks — `type(obj)`
    often reports the interface (``IGH_DocumentObject``) rather than the
    concrete class, and `isinstance` against the concrete class then fails.
    Comparing the .NET type name is reliable.
    """
    return obj.GetType().Name == "GH_NumberSlider"


def _strip_slider_sources(target_param, doc):
    """Remove every `GH_NumberSlider` currently feeding ``target_param`` and
    delete the slider objects from ``doc``. Non-slider sources are left alone.

    Uses the ``RemoveSource(Guid)`` overload: the ``RemoveSource(IGH_Param)``
    overload is a silent no-op in Rhino 8 CPython (presumably a marshalling
    quirk), which previously let sliders accumulate +8 per re-build cycle.
    """
    slider_objs = [s for s in list(target_param.Sources) if _is_number_slider(s)]
    for slider in slider_objs:
        target_param.RemoveSource(slider.InstanceGuid)
        try:
            doc.RemoveObject(slider, True)
        except Exception:
            pass


def ensure_joint_sliders(component, robot_model, input_name="joints", signature_key="joint_signature"):
    """Wire one Number Slider per configurable joint into the named list input.

    On first call (or when the model's joint signature changes), this helper
    deletes any previously auto-wired sliders on the target input and creates
    a fresh bank — one `GH_NumberSlider` per configurable joint, in canonical
    order, with ranges set to each joint's limits. All sliders are wired to
    a single list-access input named ``input_name``.

    No reading of slider values is necessary: the calling script declares
    ``input_name`` in its `RunScript` signature and receives the values as a
    plain Python list of floats via the standard GH solve path.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        Pass `ghenv.Component`.
    robot_model : compas_robots.RobotModel
        The model to derive joint metadata from.
    input_name : str, optional
        Name of the static list-access input the sliders are wired into.
        Must exist on the component. Defaults to ``"joints"``.
    signature_key : str, optional
        Sticky key suffix used to remember the last installed joint signature
        on this component. Override if the same component manages multiple
        slider banks.

    Returns
    -------
    list of (joint_name, joint_type, default_value) tuples in canonical
    joint order. Use this for the configuration's `joint_names` /
    `joint_types`, and as a fallback when the `joints` list isn't yet
    populated (e.g. the very first solve before the scheduled follow-up).
    """
    from compas_ghpython import create_id
    from scriptcontext import sticky as st

    doc = component.OnPingDocument()
    if doc is None:
        return []

    target_param = None
    for p in component.Params.Input:
        if p.Name == input_name:
            target_param = p
            break
    if target_param is None:
        return []

    signature = _joint_signature(robot_model)
    sig_sticky_key = create_id(component, signature_key)
    busy_sticky_key = create_id(component, signature_key + "__busy")
    last_signature = st.get(sig_sticky_key)

    configurable_joints = list(robot_model.get_configurable_joints())

    # Re-entrance guard. `doc.AddObject` and `Params.OnParametersChanged`
    # below trigger nested solves of this same component. Without this
    # guard, each nested call sees the still-stale sticky signature, enters
    # the rebuild branch again, and adds another N sliders per joint per
    # nesting level — slider counts explode (6 joints × 9 nested calls = 54).
    if st.get(busy_sticky_key):
        return [(j.name, j.type, _resolve_limits(j)[2]) for j in configurable_joints]

    # Rebuild whenever the joint signature changes OR no sliders are currently
    # wired (covers the fresh-drop case after the script has cached a
    # signature in sticky from a previous session/state).
    has_any_slider_source = any(_is_number_slider(s) for s in target_param.Sources)
    if signature != last_signature or not has_any_slider_source:
        # IMPORTANT: defer ALL mutation (strip, create, wire,
        # OnParametersChanged, expire self) to a `ScheduleSolution(delay,
        # callback)` callback — the callback fires from GH's timer thread
        # between solves, NOT inside an active solve. Doing this work inline
        # in `RunScript` (we used to) mutates the document mid-solve and
        # expires every already-queued downstream consumer, popping a
        # "X expired during a solution" breakpoint per consumer (6–8 times
        # on a robot switch). Deferring the work absorbs the cascade into
        # the upcoming solve rather than disrupting the current one.
        def _rebuild(_doc):
            if st.get(busy_sticky_key):
                return
            st[busy_sticky_key] = True
            try:
                _strip_slider_sources(target_param, doc)

                param_pivot = target_param.Attributes.Pivot
                row_height = 32
                slider_x = param_pivot.X - 260
                slider_y_top = param_pivot.Y - (len(configurable_joints) - 1) * row_height / 2

                for index, joint in enumerate(configurable_joints):
                    lower, upper, default = _resolve_limits(joint)
                    slider_pivot = System.Drawing.PointF(slider_x, slider_y_top + index * row_height)
                    slider = _create_slider(joint, lower, upper, default, slider_pivot)
                    doc.AddObject(slider, False)
                    target_param.AddSource(slider)

                st[sig_sticky_key] = signature
                component.Params.OnParametersChanged()
                # The callback fires right before the scheduled solve, so
                # expiring here causes that solve to re-run this component
                # with the freshly-wired sliders flowing in.
                component.ExpireSolution(False)
            finally:
                st[busy_sticky_key] = False

        doc.ScheduleSolution(5, _rebuild)

    return [(j.name, j.type, _resolve_limits(j)[2]) for j in configurable_joints]
