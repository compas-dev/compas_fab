from __future__ import absolute_import


def create_id(component, name):
    return "{}_{}".format(name, component.InstanceGuid)
