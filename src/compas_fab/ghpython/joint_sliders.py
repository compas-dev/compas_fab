"""Helper for auto-creating per-joint Number Sliders and dynamic input params.

Mirrors the spirit of `ensure_value_list` but for the more involved case where
the input set itself depends on runtime data (the number, names, and limits of
configurable joints in a `RobotModel`).
"""

import math

import compas

if compas.RHINO:
    import Grasshopper
    import System


_JOINT_INPUT_PREFIX = "j_"
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


def _remove_joint_inputs(component, doc):
    """Tear down every existing `j_*` input parameter and the auto-created
    sliders feeding them."""
    to_remove = [p for p in component.Params.Input if p.Name.startswith(_JOINT_INPUT_PREFIX)]
    for param in to_remove:
        for source in list(param.Sources):
            param.RemoveSource(source)
            if isinstance(source, Grasshopper.Kernel.Special.GH_NumberSlider):
                try:
                    doc.RemoveObject(source, True)
                except Exception:
                    pass
        component.Params.UnregisterInputParameter(param, True)
    if to_remove:
        component.Params.OnParametersChanged()


def _create_slider(joint, lower, upper, default, pivot):
    slider = Grasshopper.Kernel.Special.GH_NumberSlider()
    slider.CreateAttributes()
    # SetInitCode "min<value<max" is the documented shortcut to configure range + value in one shot.
    slider.SetInitCode("{:.6f}<{:.6f}<{:.6f}".format(lower, default, upper))
    slider.NickName = joint.name
    slider.Attributes.Pivot = pivot
    slider.Attributes.ExpireLayout()
    return slider


def _create_input_param(joint, description):
    param = Grasshopper.Kernel.Parameters.Param_Number()
    param.Name = "{}{}".format(_JOINT_INPUT_PREFIX, joint.name)
    param.NickName = joint.name
    param.Description = description
    param.Access = Grasshopper.Kernel.GH_ParamAccess.item
    param.Optional = True
    return param


def _read_input_value(param, default):
    """Pull a single float out of an input parameter's volatile data."""
    data = param.VolatileData
    if data.PathCount == 0:
        return default
    for path_index in range(data.PathCount):
        branch = data.get_Branch(path_index)
        for item in branch:
            if item is None:
                continue
            try:
                return float(item.Value)
            except (AttributeError, TypeError, ValueError):
                try:
                    return float(item)
                except (TypeError, ValueError):
                    continue
    return default


def ensure_joint_sliders(component, robot_model, signature_key="joint_signature"):
    """Sync per-joint inputs and sliders to a `RobotModel`.

    On first call (or when the model's joint signature changes) the helper
    registers one `Param_Number` input per configurable joint, auto-creates
    a `GH_NumberSlider` feeding each input with the joint's limits, and
    schedules a follow-up solve so the freshly wired values reach the
    component right away.

    Subsequent calls with the same model are no-ops aside from reading the
    current slider values.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        Pass `ghenv.Component`.
    robot_model : compas_robots.RobotModel
        The model to derive joint metadata from.
    signature_key : str, optional
        Sticky key suffix used to remember the last installed joint signature
        on this component. Override if the same component manages multiple
        slider banks.

    Returns
    -------
    list of (joint_name, joint_type, value) tuples in `robot_model.get_configurable_joints()` order.
    """
    from compas_ghpython import create_id
    from scriptcontext import sticky as st

    doc = component.OnPingDocument()
    if doc is None:
        return []

    signature = _joint_signature(robot_model)
    sticky_key = create_id(component, signature_key)
    last_signature = st.get(sticky_key)

    if signature != last_signature:
        _remove_joint_inputs(component, doc)

        component_pivot = component.Attributes.Pivot
        slider_x = component_pivot.X - 280
        slider_y_top = component_pivot.Y + 20
        row_height = 32

        configurable_joints = robot_model.get_configurable_joints()
        for index, joint in enumerate(configurable_joints):
            lower, upper, default = _resolve_limits(joint)
            description = "{} [{:.3f} .. {:.3f}]".format(joint.name, lower, upper)
            param = _create_input_param(joint, description)
            insert_index = len(component.Params.Input)
            component.Params.RegisterInputParam(param, insert_index)

            slider_pivot = System.Drawing.PointF(slider_x, slider_y_top + index * row_height)
            slider = _create_slider(joint, lower, upper, default, slider_pivot)
            doc.AddObject(slider, False)
            param.AddSource(slider)

        component.Params.OnParametersChanged()
        st[sticky_key] = signature
        doc.ScheduleSolution(5)
        # Sliders are wired but won't deliver values until the scheduled solve runs.
        return [(j.name, j.type, _resolve_limits(j)[2]) for j in configurable_joints]

    # Same signature — read current values from the existing inputs.
    values = []
    inputs_by_name = {p.Name: p for p in component.Params.Input}
    for joint in robot_model.get_configurable_joints():
        param = inputs_by_name.get("{}{}".format(_JOINT_INPUT_PREFIX, joint.name))
        _, _, default = _resolve_limits(joint)
        value = _read_input_value(param, default) if param is not None else default
        values.append((joint.name, joint.type, value))
    return values
