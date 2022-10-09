from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
from compas.artists import Artist
from compas.plugins import plugin

from compas_fab.robots import ReachabilityMap

if compas.RHINO:
    from compas_ghpython.artists import RobotModelArtist

    from .reachability_artist import ReachabilityMapArtist

    __all__ = [
        'ReachabilityMapArtist',
    ]

    @plugin(category='factories', requires=['Rhino'])
    def register_artists():
        Artist.register(ReachabilityMap, ReachabilityMapArtist, context='Grasshopper')

    # deprecated artists (aliased for backwards compat)
    __all__ += [
        'RobotArtist',  # deprecated
        'RobotModelArtist',  # deprecated
    ]

    RobotArtist = RobotModelArtist
