"""Helpers for auto-creating Grasshopper Value Lists on component inputs.

Two flavours:

* :func:`ensure_value_list` — fire-and-forget, used for static option sets
  (e.g. the fixed list of robots in `RobotCellLibrary`). Creates the VL on
  first solve when nothing is wired, then never touches it again.

* :func:`ensure_dynamic_value_list` — for options that depend on upstream
  data (e.g. the keys of `cell.tool_models`). Tracks the VL we created
  via `scriptcontext.sticky` and refreshes its items when the option set
  changes. Defers the canvas mutation to a `ScheduleSolution(delay,
  callback)` callback so the rebuild happens between solves rather than
  expiring downstream consumers mid-solve.
"""

import compas

if compas.RHINO:
    import Grasshopper
    import System


def ensure_value_list(component, input_name, options, default=None, x_offset=240, y_offset=0):
    """Create and wire a Value List to a component input if nothing is connected.

    No-op if the input already has a source connected. Safe to call on every
    `RunScript` invocation: Grasshopper drops the duplicate-add silently and the
    `SourceCount > 0` check handles subsequent solves.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        The hosting component. Pass `ghenv.Component`.
    input_name : str
        The `Name` of the input parameter to attach to.
    options : iterable of str
        The string options to populate the Value List with.
    default : str, optional
        If provided and present in `options`, that item is pre-selected.
    x_offset, y_offset : float, optional
        Pixel offset of the value list from the input pivot. Default places
        the value list to the left of the input.
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
    pivot = param.Attributes.Pivot
    value_list.Attributes.Pivot = System.Drawing.PointF(pivot.X - x_offset, pivot.Y + y_offset)
    value_list.Attributes.ExpireLayout()

    doc.AddObject(value_list, False)
    param.AddSource(value_list)

    # The source was wired during this solve, so the new value won't reach the
    # input until the next one. Schedule a fresh solve so the user doesn't have
    # to manually re-trigger the component.
    doc.ScheduleSolution(5)


def ensure_dynamic_value_list(component, input_name, options, signature_key=None, x_offset=240, y_offset=0):
    """Maintain a Value List on `input_name` whose items reflect `options`.

    Unlike :func:`ensure_value_list`, this variant tracks the VL it created
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
    component : Grasshopper.Kernel.IGH_Component
        Pass `ghenv.Component`.
    input_name : str
        Name of the input parameter to attach a VL to.
    options : iterable of str
        Current option set the VL should expose. Empty means "skip".
    signature_key : str, optional
        Suffix used to identify the tracked VL in sticky. Override only when
        a single component manages more than one dynamic VL.
    x_offset, y_offset : float, optional
        Pixel offset of the value list from the input pivot.
    """
    from compas_ghpython import create_id
    from scriptcontext import sticky as st

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
            pivot = target_param.Attributes.Pivot
            vl.Attributes.Pivot = System.Drawing.PointF(pivot.X - x_offset, pivot.Y + y_offset)
            vl.Attributes.ExpireLayout()
            doc.AddObject(vl, False)
            target_param.AddSource(vl)
            st[sticky_key] = str(vl.InstanceGuid)
        component.ExpireSolution(False)

    doc.ScheduleSolution(5, _rebuild)
