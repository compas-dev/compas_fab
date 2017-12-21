try:
    import Grasshopper as gh
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise


def update_component(ghenv, delay):
    """Schedule an update of the Grasshopper component.

    After the specified delay, the GH component will be automatically updated.

    Args:
        ghenv (:class:`GhPython.Component.PythonEnvironment`): just available from within the
            python Grasshopper component.

        delay (:obj:`int`): Time in milliseconds until the update is performed.
    """
    if delay <= 0:
        raise ValueError('Delay must be greater than zero')

    grasshopper_component = ghenv.Component
    grasshopper_doc = grasshopper_component.OnPingDocument()

    def callback(grasshopper_doc):
        grasshopper_component.ExpireSolution(False)

    grasshopper_doc.ScheduleSolution(delay, gh.Kernel.GH_Document.GH_ScheduleDelegate(callback))
