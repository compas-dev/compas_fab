from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

if compas.RHINO:
    from compas_rhino.artists import RobotModelArtist

    __all__ = [
        'RobotArtist',
        'RobotModelArtist',
    ]

    # deprecated alias
    RobotArtist = RobotModelArtist
