"""Helpers for auto-creating Grasshopper Value Lists on component inputs.

Two flavours, `ensure_value_list` (static option sets) and
`ensure_dynamic_value_list` (options that track upstream data) — see the
`compas_fab.ghpython` package overview for how they differ and when to use each.
"""

from typing import Iterable
from typing import Optional

import Grasshopper
import System
from compas_ghpython import create_id
from scriptcontext import sticky as st


def _place_left_of_input(
    obj: Grasshopper.Kernel.IGH_DocumentObject,
    param: Grasshopper.Kernel.IGH_Param,
    gap: float = 30.0,
    y_offset: float = 0.0,
) -> None:
    """Position a floating object just left of a component input.

    Places ``obj`` so its right edge sits ``gap`` pixels to the left of the
    input's pivot and its vertical centre lines up with the input, regardless of
    the object's width or how its pivot relates to its bounds.

    Must be called once ``obj`` is in the document AND between solves (e.g. from a
    ``ScheduleSolution`` callback): ``PerformLayout`` only yields the real rendered
    size there. The shift is computed from the measured ``Bounds`` rather than from
    the pivot directly, because a value list's pivot is *not* the top-left of its
    bounds (the NickName label offsets it).
    """
    obj.Attributes.PerformLayout()
    bounds = obj.Attributes.Bounds
    pivot = obj.Attributes.Pivot
    target = param.Attributes.Pivot
    dx = (target.X - gap) - (bounds.X + bounds.Width)
    dy = (target.Y + y_offset) - (bounds.Y + bounds.Height / 2.0)
    obj.Attributes.Pivot = System.Drawing.PointF(pivot.X + dx, pivot.Y + dy)
    obj.Attributes.ExpireLayout()


def ensure_boolean_toggle(
    component: Grasshopper.Kernel.IGH_Component,
    input_name: str,
    default: bool = True,
    x_offset: float = 30,
    y_offset: float = 0,
) -> None:
    """Create and wire a Boolean Toggle to a component input if nothing is connected.

    No-op if the input already has a source connected. Safe to call on every
    `RunScript` invocation. Gives a boolean input a one-click on/off widget on the
    canvas instead of forcing the user to find and wire a separate toggle.

    Parameters
    ----------
    component
        The hosting component. Pass `ghenv.Component`.
    input_name
        The `Name` of the input parameter to attach to.
    default
        The toggle's initial value. Defaults to True.
    x_offset, y_offset
        `x_offset` is the gap (px) between the toggle's right edge and the input;
        `y_offset` nudges it vertically. Placed just to the left of the input,
        vertically centred on it.
    """
    param = None
    for p in component.Params.Input:
        if p.Name == input_name:
            param = p
            break
    if param is None or param.SourceCount > 0:
        return

    doc = component.OnPingDocument()
    if doc is None:
        return

    toggle = Grasshopper.Kernel.Special.GH_BooleanToggle()
    toggle.NickName = input_name
    toggle.Value = bool(default)

    toggle.CreateAttributes()
    # Rough placement so it lands somewhere sane until the precise reposition.
    toggle.Attributes.Pivot = param.Attributes.Pivot
    doc.AddObject(toggle, False)
    param.AddSource(toggle)

    # As with the value list, the real rendered size is only available between
    # solves, so position it precisely in a scheduled callback. That callback
    # also serves as the follow-up solve that lets the toggle's value reach the
    # input, so the user doesn't have to re-trigger the component.
    def _finish(_doc):
        _place_left_of_input(toggle, param, gap=x_offset, y_offset=y_offset)

    doc.ScheduleSolution(5, _finish)


def ensure_value_list(
    component: Grasshopper.Kernel.IGH_Component,
    input_name: str,
    options: Iterable[str],
    default: Optional[str] = None,
    x_offset: float = 30,
    y_offset: float = 0,
) -> None:
    """Create and wire a Value List to a component input if nothing is connected.

    No-op if the input already has a source connected. Safe to call on every
    `RunScript` invocation: Grasshopper drops the duplicate-add silently and the
    `SourceCount > 0` check handles subsequent solves.

    Parameters
    ----------
    component
        The hosting component. Pass `ghenv.Component`.
    input_name
        The `Name` of the input parameter to attach to.
    options
        The string options to populate the Value List with.
    default
        If provided and present in `options`, that item is pre-selected.
    x_offset, y_offset
        `x_offset` is the gap (px) between the value list's right edge and the
        input; `y_offset` nudges it vertically. The value list is placed just to
        the left of the input, vertically centred on it.
    """
    param = None
    for p in component.Params.Input:
        if p.Name == input_name:
            param = p
            break
    if param is None or param.SourceCount > 0:
        return

    doc = component.OnPingDocument()
    if doc is None:
        return

    value_list = Grasshopper.Kernel.Special.GH_ValueList()
    value_list.NickName = input_name
    value_list.ListItems.Clear()
    for opt in options:
        value_list.ListItems.Add(Grasshopper.Kernel.Special.GH_ValueListItem(opt, '"{}"'.format(opt)))
    if default is not None:
        for i, opt in enumerate(options):
            if opt == default:
                value_list.SelectItem(i)
                break

    value_list.CreateAttributes()
    # Rough placement so it lands somewhere sane until the precise reposition.
    value_list.Attributes.Pivot = param.Attributes.Pivot
    doc.AddObject(value_list, False)
    param.AddSource(value_list)

    # The value list's real width isn't available during this solve
    # (`PerformLayout` only reports the rendered size between solves), so position
    # it precisely in a scheduled callback. That callback doubles as the follow-up
    # solve which lets the selected value reach the input, so the user doesn't have
    # to re-trigger the component.
    def _finish(_doc):
        _place_left_of_input(value_list, param, gap=x_offset, y_offset=y_offset)

    doc.ScheduleSolution(5, _finish)


def ensure_dynamic_value_list(
    component: Grasshopper.Kernel.IGH_Component,
    input_name: str,
    options: Iterable[str],
    signature_key: Optional[str] = None,
    x_offset: float = 30,
    y_offset: float = 0,
) -> None:
    """Maintain a Value List on `input_name` whose items reflect `options`.

    Unlike [`ensure_value_list`][], this variant tracks the VL it created
    (sticky-cached by the component's `InstanceGuid` and `signature_key`) and
    re-populates its items whenever the `options` set changes between solves.
    Use this when the available choices come from upstream data — e.g. the
    keys of `cell.tool_models` for a "pick a tool" dropdown.

    Behaviour:

    * If a non-VL source is already wired to `input_name`, the helper does
      nothing — the user provided their own value.
    * If a VL is wired AND we tracked it AND the items don't match
      `options`, the items are refreshed.
    * If nothing is wired, a fresh VL is created and added to the canvas,
      then a follow-up solve is scheduled so the value flows on the next
      pass.

    All mutation (creating the VL, updating items, expiring the component)
    runs inside a `doc.ScheduleSolution(delay, callback)` callback to avoid
    "X expired during a solution" warnings on downstream consumers.

    Parameters
    ----------
    component
        Pass `ghenv.Component`.
    input_name
        Name of the input parameter to attach a VL to.
    options
        Current option set the VL should expose. Empty means "skip".
    signature_key
        Suffix used to identify the tracked VL in sticky. Override only when
        a single component manages more than one dynamic VL.
    x_offset, y_offset
        `x_offset` is the gap (px) between the value list's right edge and the
        input; `y_offset` nudges it vertically. Placed just left of the input.
    """
    doc = component.OnPingDocument()
    if doc is None:
        return

    target_param = next((p for p in component.Params.Input if p.Name == input_name), None)
    if target_param is None:
        return

    options = list(options)
    if not options:
        return

    sticky_key = create_id(component, "dvl_" + (signature_key or input_name))
    tracked_guid = st.get(sticky_key)

    # Look for our tracked VL among the input's sources.
    existing_vl = None
    for source in target_param.Sources:
        if str(source.InstanceGuid) == tracked_guid and source.GetType().Name == "GH_ValueList":
            existing_vl = source
            break

    # If something else is wired (user's own source, or a VL we don't track),
    # stay out of the way.
    foreign_sources = [
        s for s in target_param.Sources
        if existing_vl is None or str(s.InstanceGuid) != str(existing_vl.InstanceGuid)
    ]
    if existing_vl is None and foreign_sources:
        return

    if existing_vl is not None:
        current = [it.Name for it in existing_vl.ListItems]
        if current == options:
            return  # already up to date

    def _rebuild(_doc):
        if existing_vl is not None:
            previous_selection = next(
                (it.Name for it in existing_vl.ListItems if it.Selected), None
            )
            existing_vl.ListItems.Clear()
            for opt in options:
                existing_vl.ListItems.Add(Grasshopper.Kernel.Special.GH_ValueListItem(opt, '"{}"'.format(opt)))
            # Restore selection if still valid, else pick the first item.
            picked = False
            if previous_selection is not None:
                for i, opt in enumerate(options):
                    if opt == previous_selection:
                        existing_vl.SelectItem(i)
                        picked = True
                        break
            if not picked:
                existing_vl.SelectItem(0)
            existing_vl.ExpireSolution(False)
        else:
            vl = Grasshopper.Kernel.Special.GH_ValueList()
            vl.NickName = input_name
            vl.ListItems.Clear()
            for opt in options:
                vl.ListItems.Add(Grasshopper.Kernel.Special.GH_ValueListItem(opt, '"{}"'.format(opt)))
            vl.SelectItem(0)
            vl.CreateAttributes()
            vl.Attributes.Pivot = target_param.Attributes.Pivot
            doc.AddObject(vl, False)
            _place_left_of_input(vl, target_param, gap=x_offset, y_offset=y_offset)
            target_param.AddSource(vl)
            st[sticky_key] = str(vl.InstanceGuid)
        component.ExpireSolution(False)

    doc.ScheduleSolution(5, _rebuild)
