from importlib import import_module

MODULES = [
    'compas_robot',
    'compas_robot.rfl'
]

for name in MODULES:
    obj = import_module(name)

    print obj

    with open('source/pages/reference/{0}.rst'.format(name), 'wb+') as fp:
        fp.write(obj.__doc__)
