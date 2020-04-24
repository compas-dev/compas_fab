from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from contextlib import contextmanager
from contextlib import ExitStack

from compas_fab.backends import RosClient
from compas_fab.backends import VrepClient


__all__ = [
    'ClientManager',
]

SUPPORTED_BACKENDS = {
    'ros': RosClient,
    'vrep': VrepClient,
}


class MultiResourceManager(ExitStack):
    def __init__(self):
        super(MultiResourceManager, self).__init__()
        self.resources = {}
        self.wrappers = {}

    def check_resource_ok(self, resource):
        return True

    @contextmanager
    def _cleanup_on_error(self):
        with ExitStack() as stack:
            stack.push(self)
            yield
            # The validation check passed and didn't raise an exception
            # Accordingly, we want to keep the resource, and pass it
            # back to our caller
            stack.pop_all()

    def enter_context(self, resource):
        wrapped = super(MultiResourceManager, self).enter_context(resource)
        if not self.check_resource_ok(wrapped):
            msg = "Failed validation for {!r}"
            raise RuntimeError(msg.format(resource))
        return wrapped

    def __enter__(self):
        with self._cleanup_on_error():
            self.wrappers = {r_name: self.enter_context(r) for r_name, r in self.resources.items()}
        return self.wrappers


class ClientManager(MultiResourceManager):
    def __init__(self, clients):
        super(ClientManager, self).__init__()
        for client_name in SUPPORTED_BACKENDS:
            if client_name in clients:
                self.__setattr__(client_name, SUPPORTED_BACKENDS[client_name]())
            else:
                self.__setattr__(client_name, None)

    def __enter__(self):
        self.resources = {
            client_name: client for client_name, client in self.__dict__.items()
            if client is not None and client_name in SUPPORTED_BACKENDS
        }
        super(ClientManager, self).__enter__()
        return self
