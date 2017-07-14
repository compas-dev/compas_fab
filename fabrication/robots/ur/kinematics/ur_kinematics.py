'''
Created on 06.07.2017

@author: rustr
'''

from ur_kin_ros import forward_ros, inverse_ros
import math
from compas_fabrication.fabrication.geometry import Frame


def inverse_kinematics(frame, ur_params, q6_des = 0.0):
    """ Inverse kinematics function.

    This is the wrapper for the inverse function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need 
    to swap angles and rotate the first joint by -pi. (The initial position can 
    be visualized by loading the meshes.)
    
    Args:
        the frame to reach.
        ur_params: UR defined parameters for the model
        q6_des, an optional parameter which designates what the q6 value 
        should take, in case of an infinite solution on that joint.   
        
    Returns:
        q_sols, an 8x6 array of doubles returned, 8 possible q joint 
        solutions, all angles should be in [0,2 * pi]
                
    """
    
    print frame
    print ur_params
    
    T = [0 for i in range(16)]
    
    T[0], T[4], T[8] = list(frame.zaxis)
    T[1], T[5], T[9] = list(frame.xaxis)
    T[2], T[6], T[10] = list(frame.yaxis)
    T[3], T[7], T[11] = list(frame.point)
    
    try:
        qsols = inverse_ros(T, ur_params, q6_des)
        for i in range(len(qsols)):
            qsols[i][0] -= math.pi
        return qsols
    except ZeroDivisionError:
        return []


def forward_kinematics(configuration, ur_params):
    """ Forward kinematics function.
    
    This is the wrapper for the forward function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need 
    to swap angles and rotate the first joint by -pi. (The initial position can 
    be visualized by loading the meshes.)
    
    Args:
        configuration, the 6 joint angles in radians 
        ur_params: UR defined parameters for the model
    Returns:    
        the frame        
    """
    
    configuration[0] += math.pi
    
    T = forward_ros(configuration, ur_params)
    
    xaxis = [T[1], T[5], T[9]]
    yaxis = [T[2], T[6], T[10]]
    point = [T[3], T[7], T[11]]
    
    return Frame(point, xaxis, yaxis)



if __name__ == "__main__":
    
    from compas.geometry.elements import Point
    frame = Frame.worldXY()
    frame.point = Point([-660.8691, -1115.0190, 360.5604])
    ur_params = [89.159, -425.0, -392.25, 109.15, 94.65, 82.3]
    
    print inverse_kinematics(frame, ur_params, q6_des = 0.0)
    
    q = [-1.225745, -1.898395, -1.429878, -2.043711, 5.444258, 2.357329]
    frame =  forward_kinematics(q, ur_params)
    print inverse_kinematics(frame, ur_params, q6_des = 0.0)
