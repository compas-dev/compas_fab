from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
import compas.plugins


@compas.plugins.plugin(category='install')
def installable_rhino_packages():
    return ['compas_fab', 'roslibpy']


@compas.plugins.plugin(category='install')
def after_rhino_install(installed_packages):
    # GH Components are not supported on non-Windows platforms
    if not compas.is_windows():
        return []

    if 'compas_fab' not in installed_packages:
        return []

    import compas_fab.ghpython.components.install
    return compas_fab.ghpython.components.install.install()


@compas.plugins.plugin(category='install')
def after_rhino_uninstall(installed_packages):
    # GH Components are not supported on non-Windows platforms
    if not compas.is_windows():
        return []

    if 'compas_fab' not in installed_packages:
        return []

    import compas_fab.ghpython.components.install
    return compas_fab.ghpython.components.install.uninstall()


if __name__ == "__main__":
    print('This installation method is obsolete.')
    print('Please use `python -m compas_rhino.install` instead')
    print('COMPAS FAB will be automatically detected and installed')
