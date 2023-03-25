from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
import compas.plugins


@compas.plugins.plugin(category="install")
def installable_rhino_packages():
    return ["compas_fab", "roslibpy"]
