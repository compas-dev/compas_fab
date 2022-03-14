from __future__ import print_function

import platform

import compas

import compas_fab

if __name__ == '__main__':
    print()
    print('Yay! COMPAS FAB is installed correctly!')
    print()
    print('COMPAS FAB: {}'.format(compas_fab.__version__))
    print('COMPAS: {}'.format(compas.__version__))
    print('Python: {} ({})'.format(platform.python_version(), platform.python_implementation()))
