********************************************************************************
Planning scene and collision objects
********************************************************************************

To plan motion paths that avoid collisions with other objects than the robot
itself, the backend's planning scene has to be updated. 

This is the representation of the planning scene in RViz with the UR5.

.. figure:: 04_collision_objects_attached_without.jpg
    :figclass: figure
    :class: figure-img img-fluid

Collision meshes
================

The following script adds a floor to the planning scene.

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


The backend's updated planning scene after executing the above script.

.. figure:: 04_collision_objects.jpg
    :figclass: figure
    :class: figure-img img-fluid


The following script adds several boxes (bricks) to the planning scene. Here, 
we use ``append`` instead of ``add`` to have multiple collision objects with the same
identifier.

.. code-block:: python

    import time
    import compas_fab
    from compas.geometry import Box
    from compas.datastructures import Mesh
    from compas_fab.robots.ur5 import Robot
    from compas_fab.robots import PlanningScene
    from compas_fab.robots import CollisionMesh
    from compas_fab.backends import RosClient

    client = RosClient()
    client.run()
    robot = Robot(client)
    
    scene = PlanningScene(robot)

    brick = Box.from_width_height_depth(0.11, 0.07, 0.25)
    mesh = Mesh.from_vertices_and_faces(brick.vertices, brick.faces)
    cm = CollisionMesh(mesh, 'brick')
    cm.frame.point.y += 0.3

    for i in range(5):
        cm.frame.point.z += brick.zsize
        scene.append_collision_mesh(cm)
    
    time.sleep(1.) #sleep a bit before terminating the client
    client.close()
    client.terminate()


The backend's updated planning scene after executing the above script. Note the 
red robot link indicating the collision.    

.. figure:: 04_collision_objects_append.jpg
    :figclass: figure
    :class: figure-img img-fluid



Attach a collision mesh to a robot's end-effector
=================================================

The following script attaches a collision mesh to the robot's end-effector.
Collision objects can attached to any of the robot's links.

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

The backend's updated planning scene after executing the above script.

.. figure:: 04_collision_objects_attached.jpg
    :figclass: figure
    :class: figure-img img-fluid



Plan motion with an attached collision mesh
===========================================

Coming soon...
