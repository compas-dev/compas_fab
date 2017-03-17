from importlib import import_module

MODULES = [
    'compas_fabrication',
    'compas_fabrication.fabrication.robots.rfl',
    'compas_fabrication.fabrication.grasshopper'
]

if __name__ == '__main__':
    for name in MODULES:
        obj = import_module(name)

        print obj

        try:
            with open('source/pages/reference/{0}.rst'.format(name), 'wb+') as fp:
                fp.write(obj.__doc__)
        except:
            print('WARN: File cannot be opened: "source/pages/reference/{0}.rst"'.format(name))
