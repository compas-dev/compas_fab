********************************************************************************
Plan motion
********************************************************************************

There are 2 function that allow to plan a robotic movement without collisions:
`plan_cartesian_motion` and `plan_motion`.
More coming soon ...


Plan cartesian motion
=====================

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.backends import RosClient
    from compas_fab.robots.ur5 import Robot

    with RosClient() as client:
        
        robot = Robot(client)

        frames = []
        frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
        frames.append(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]))
        start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])

        response = robot.plan_cartesian_motion(frames,
                                            start_configuration,
                                            max_step=0.01,
                                            avoid_collisions=True)
        
        print("Computed cartesian path with %d configurations, " % len(response.configurations))
        print("following %d%% of requested trajectory." % (response.fraction * 100))
        print("Executing this path at full speed would take approx. %.3f seconds." % response.time_from_start)


Plan motion
===========
In contrast to the cartesian path, the `plan_motion` allows to describe the goal
with constraints rather than defined frames.

.. code-block:: python

    import math
    from compas.geometry import Frame
    from compas_fab.robots import Configuration
    from compas_fab.backends import RosClient
    from compas_fab.robots.ur5 import Robot

    with RosClient() as client:
        
        robot = Robot(client)
    
        frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        tolerance_position = 0.001
        tolerance_axes = [math.radians(1)] * 3
        
        start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        group = robot.main_group_name
        
        # create goal constraints from frame
        goal_constraints = robot.constraints_from_frame(frame,
                                                        tolerance_position,
                                                        tolerance_axes,
                                                        group)

        response = robot.plan_motion(goal_constraints, 
                                    start_configuration,
                                    group,
                                    planner_id='RRT')

        print("Computed kinematic path with %d configurations." % len(response.configurations))
        print("Executing this path at full speed would take approx. %.3f seconds." % response.time_from_start)
