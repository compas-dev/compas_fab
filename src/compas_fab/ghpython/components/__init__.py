import compas_fab

assembly_name = 'COMPAS FAB'
assembly_version = compas_fab.__version__.split('-')[0]
filename = '{}_{}.ghpy'.format(assembly_name, assembly_version)


def create_id(component, name):
    return '{}_{}'.format(name, component.InstanceGuid)

def coerce_frame(plane):
    import Rhino
    from compas.geometry import Frame
    if isinstance(plane, Rhino.Geometry.Plane):
        return Frame(plane.Origin, plane.XAxis, plane.YAxis)
    elif isinstance(plane, Frame):
        return plane
    else:
        return Frame(*plane)