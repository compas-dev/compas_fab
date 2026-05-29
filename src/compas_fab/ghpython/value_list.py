"""Helper for auto-creating a Grasshopper Value List on an unconnected input."""

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
