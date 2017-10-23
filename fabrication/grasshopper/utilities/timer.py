import Grasshopper as gh

def gh_component_timer(ghenv, interval):
    """Calling this function from within a python Grasshopper component updates 
        the component after the given interval in ms.
    
    Args:
        ghenv (GhPython.Component.PythonEnvironment): just available from within the 
        python Grasshopper component
        
        interval (int): the time until the update is performed in milliseconds.
    """
    if interval <= 0: interval = 1
    ghComp = ghenv.Component
    ghDoc = ghComp.OnPingDocument()
    def callBack(ghDoc):
        ghComp.ExpireSolution(False)
    ghDoc.ScheduleSolution(interval, gh.Kernel.GH_Document.GH_ScheduleDelegate(callBack))