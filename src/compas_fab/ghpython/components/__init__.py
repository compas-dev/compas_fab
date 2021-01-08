assembly_name = "COMPAS FAB"
assembly_version = "0.15.0"
filename = "%s_%s.ghpy" % (assembly_name, assembly_version)


def create_id(component, name):
    return '%s_%s' % (name, component.InstanceGuid)
