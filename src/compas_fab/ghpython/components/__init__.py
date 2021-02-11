assembly_name = 'COMPAS FAB'
assembly_version = '0.15.0'
filename = '{}_{}.ghpy'.format(assembly_name, assembly_version)


def create_id(component, name):
    return '{}_{}'.format(name, component.InstanceGuid)
