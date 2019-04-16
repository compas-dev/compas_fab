import logging

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.robots import *
from compas_fab.robots import rfl
from compas_fab.backends import VrepClient

# Configure logging to DEBUG to see detailed timing of the path planning
logging.basicConfig(level=logging.DEBUG)

# Configure parameters for path planning
start_pose      = Frame((7.453, 2.905, 0.679), (1, 0, 0), (0, -1, 0))
goal_pose       = Frame((5.510, 5.900, 1.810), (0, 0, -1), (0, 1, 0))
algorithm       = 'rrtconnect'
max_trials      = 1
resolution      = 0.02
building_member = Mesh.from_obj(compas_fab.get('planning_scene/timber_beam.obj'))
structure       = [Mesh.from_obj(compas_fab.get('planning_scene/timber_structure.obj'))]
metric          = [0.1] * 9
fast_search     = True

with VrepClient(debug=True) as client:
    robot = rfl.Robot('A', client=client)
    client.pick_building_member(robot, building_member, start_pose)

    path = client.plan_motion(robot,
                              goal_pose,
                              metric_values=metric,
                              collision_meshes=structure,
                              algorithm=algorithm,
                              trials=max_trials,
                              resolution=resolution,
                              shallow_state_search=fast_search)

    print('Found path of %d steps' % len(path))
