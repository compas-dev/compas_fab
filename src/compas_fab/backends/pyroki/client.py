from compas_fab.backends.interfaces import ClientInterface

from .problem_cache import PyRokiProblemCache


class PyRokiClient(ClientInterface):
    """In-process client holding FAB state and a compiled PyRoKI model."""

    def __init__(self):
        super(PyRokiClient, self).__init__()
        self._pyroki_model = None
        self._problem_cache = PyRokiProblemCache()

    @property
    def pyroki_model(self):
        return self._pyroki_model
