from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas.plugins


@compas.plugins.plugin(category='install')
def installable_rhino_packages():
    return ['compas_fab', 'roslibpy']


if __name__ == "__main__":
    print('This installation method is obsolete.')
    print('Please use `python -m compas_rhino.install` instead')
    print('COMPAS FAB will be automatically detected and installed')
