********************************************************************************
Forward and inverse kinematics
********************************************************************************

Forward Kinematics
==================

The Forward Kinematics function/algorithm takes the joint states, or configuration,
as the input, and calculates the pose of the end effector in the task space
as the output. This means the state of each joint in the articulated body
of a robot needs to be defined.

Joint states are described in **compas_fab** with the
:class:`compas_fab.robots.Configuration` class.

.. code-block:: python

    from compas_fab.backends import RosClient
    from compas_fab.robots import Configuration
    from compas_fab.robots.ur5 import Robot
    
    client = RosClient()
    client.run()
    robot = Robot(client)
    
    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.000])

    response = robot.forward_kinematics(configuration)
    print("Frame in the robot's coordinate system", response.frame_RCF)
    print("Frame in the world coordinate system", response.frame_WCF)

    client.close()
    client.terminate()

Inverse Kinematics
==================

Inverse Kinematics is the inverse function/algorithm of Forward Kinematics. The
Forward Kinematics function/algorithm takes a target end-effector pose in the
task space as the input, and calculates the joint states required for the
end effector to reach the target pose. The output of an inverse kinematics are
the joint states, i.e. the configuration of the robot.

The following code exemplifies how to calculate this:

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.backends import RosClient
    from compas_fab.robots.ur5 import Robot

    client = RosClient()
    client.run()
    robot = Robot(client)
    
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.init_configuration()
    
    configuration = robot.inverse_kinematics(frame_WCF, start_configuration)
    print("Found configuration", configuration)

    client.close()
    client.terminate()