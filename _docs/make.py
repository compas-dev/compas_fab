from importlib import import_module

MODULES = [
    'compas_fab',
    'compas_fab.fab.geometry',
    'compas_fab.fab.robots',
    'compas_fab.fab.robots.rfl',
    'compas_fab.fab.ghpython'
]

if __name__ == '__main__':
    for name in MODULES:
        obj = import_module(name)

        print(obj)

        try:
            with open('source/pages/reference/{0}.rst'.format(name), 'w+') as fp:
                fp.write(obj.__doc__)
        except:
            print('WARN: File cannot be opened: "source/pages/reference/{0}.rst"'.format(name))
