********************************************************************************
Planning scene and collision objects
********************************************************************************

.. figure:: 04_collision_objects_attached_without.jpg
    :figclass: figure
    :class: figure-img img-fluid

    The UR5 in rviz.

Collision meshes
================

.. code-block:: python

    import time
    import compas_fab
    from compas.datastructures import Mesh
    from compas_fab.robots.ur5 import Robot
    from compas_fab.robots import PlanningScene
    from compas_fab.robots import CollisionMesh
    from compas_fab.backends import RosClient

    client = RosClient()
    client.run()
    robot = Robot(client)
    
    scene = PlanningScene(robot)
    mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    cm = CollisionMesh(mesh, 'floor')
    scene.add_collision_mesh(cm)
    
    time.sleep(1.) #sleep a bit before terminating the client
    client.close()
    client.terminate()


.. figure:: 04_collision_objects.jpg
    :figclass: figure
    :class: figure-img img-fluid

    The representation of the backend's planning scene with the added collision mesh.


Attach a collision mesh to a robot's end-effector
=================================================

.. code-block:: python

    import time
    import compas_fab
    from compas.datastructures import Mesh
    from compas_fab.robots.ur5 import Robot
    from compas_fab.robots import PlanningScene
    from compas_fab.robots import CollisionMesh
    from compas_fab.backends import RosClient
    
    client = RosClient()
    client.run()
    robot = Robot(client)
    
    scene = PlanningScene(robot)
    # create collison object
    mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    cm = CollisionMesh(mesh, 'tip')
    # attach it to the end-effector
    group = robot.main_group_name
    scene.attach_collision_mesh_to_robot_end_effector(cm, group=group)

    time.sleep(2) #sleep a bit before terminating the client
    client.close()
    client.terminate()


.. figure:: 04_collision_objects_attached.jpg
    :figclass: figure
    :class: figure-img img-fluid

    The representation of the backend's planning scene with the attached collision mesh.


Plan motion with an attached collision mesh
===========================================

Coming soon...
