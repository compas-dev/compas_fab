from __future__ import absolute_import

try:
    import Grasshopper  # type: ignore
except ImportError:
    pass


def create_id(component, name):
    return "{}_{}".format(name, component.InstanceGuid)


def warning(component, message):
    """Add a warning message to the component.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        The component instance. Pre-Rhino8 use `self`. Post-Rhino8 use `ghenv.Component`.
    message : str
        The message to display.
    """
    component.AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Warning, message)


def error(component, message):
    """Add an error message to the component.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        The component instance. Pre-Rhino8 use `self`. Post-Rhino8 use `ghenv.Component`.
    message : str
        The message to display.
    """
    component.AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Error, message)


def remark(component, message):
    """Add a remark message to the component.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        The component instance. Pre-Rhino8 use `self`. Post-Rhino8 use `ghenv.Component`.
    message : str
        The message to display.
    """
    component.AddRuntimeMessage(Grasshopper.Kernel.GH_RuntimeMessageLevel.Remark, message)


def message(component, message):
    """Add a text that will appear under the component.

    Parameters
    ----------
    component : Grasshopper.Kernel.IGH_Component
        The component instance. Pre-Rhino8 use `self`. Post-Rhino8 use `ghenv.Component`.
    message : str
        The message to display.
    """
    component.Message = message
