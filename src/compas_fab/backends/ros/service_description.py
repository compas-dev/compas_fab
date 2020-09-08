"""
Interface definition for planner backend implementations.

This is only internal, these interfaces and their implementations
are managed internally by the RosClient class.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from roslibpy import Service
from roslibpy import ServiceRequest

from compas_fab.backends.ros.exceptions import RosValidationError

__all__ = [
    'ServiceDescription'
]


class ServiceDescription(object):
    """Internal class to simplify service call code."""

    def __init__(self, name, service_type, request_class=None, response_class=None, validator=None):
        self.name = name
        self.type = service_type
        self.request_class = request_class
        self.response_class = response_class
        self.validator = validator

    def call(self, client, request, callback, errback):
        def inner_handler(response_msg):
            response_object = self.response_class.from_msg(response_msg)

            # Validate the response if there's a validator function assigned
            if self.validator:
                try:
                    self.validator(response_object)
                except Exception as e:
                    errback(RosValidationError(e, response_object))
                    return

            callback(response_object)

        if isinstance(request, tuple):
            request_msg = self.request_class(*request)
        else:
            request_msg = self.request_class(**request)

        srv = Service(client, self.name, self.type)
        srv.call(ServiceRequest(request_msg.msg),
                 callback=inner_handler, errback=errback)

    def __call__(self, client, request, callback, errback):
        return self.call(client, request, callback, errback)
