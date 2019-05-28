from __future__ import print_function

import compas.geometry as cg
import numpy as np


### PARAMETERS
### IMPORTANT: make sure all units are SI (meters and Kilograms)!

# SET MASS and CENTER OF MASS PARAMETERS FROM THE mass_and_CoM_calibration_procedure AND mass_and_CoM_estimation RESULTS,
# OR INPUT YOUR KNOWN VALUES
sensor_to_CoM_offset = cg.Vector(
    -0.03397799730572294,
    0.042647988599212705,
    0.2900206244556921)                    # [meters]
end_effector_weight = 8.559916898585978    # [Kg]

# GRAVITY
gravity_magnitude   = 9.80665               # [m/s**2]


def lists_subtraction(list_a, list_b):
    return np.subtract(list_a, list_b).tolist()
    
def get_wrench_list(FT_data):   
    if type(FT_data) is dict:
        return [
            FT_data['force']['x'],
            FT_data['force']['y'],
            FT_data['force']['z'],
            FT_data['torque']['x'],
            FT_data['torque']['y'],
            FT_data['torque']['z']]
    elif type(FT_data) is list:
        return FT_data
    else:
        print("ERROR: Input data type mismatch: not a list!", FT_data, type(FT_data))
        raise ValueError("ERROR: Input data type mismatch: not a list!")
        return None


def get_bias_compensated_wrench(FT_data, bias):
    FT_data_list = get_wrench_list(FT_data)
    return lists_subtraction(FT_data_list, bias)
    

def get_gravity_compensation_vectors(pose, pose_unit=1.0, mass=end_effector_weight, CoM_offset_vector=sensor_to_CoM_offset):
    ### frames
    #pose_unit = 1 for meters, = 0.001 for millimeters
    pose.point = [pose.point[0]*pose_unit,
                  pose.point[1]*pose_unit,
                  pose.point[2]*pose_unit]
    
    # transform gravity to FT sensor coordinate system (FTSCS)
    g_vector_WCS = cg.Vector(0.0, 0.0, -gravity_magnitude)
    FTSCS_transformation = cg.Transformation.from_frame_to_frame(pose, cg.Frame.worldXY())
    g_vector_FTSCS = g_vector_WCS.transformed(FTSCS_transformation)
    
    ### F gravity compensation
    # F = mass * gravity
    F_gravity = g_vector_FTSCS * mass
    
    ### T gravity compensation
    # T = (lever_arm * m) X g_vector_FTSCS
    T_gravity = cg.Vector( *cg.cross_vectors((CoM_offset_vector * mass), g_vector_FTSCS) )
    
    return [F_gravity.x, F_gravity.y, F_gravity.z, T_gravity.x, T_gravity.y, T_gravity.z]


def get_gravity_compensated_wrench(pose, FT_data):
    FT_data_list = get_wrench_list(FT_data)
    FT_gravity = get_gravity_compensation_vectors(pose)
    return lists_subtraction(FT_data_list, FT_gravity)
    

def get_gravity_and_bias_compensated_wrench(pose, FT_data, current_bias):
    gravity_compensated = get_gravity_compensated_wrench(pose, FT_data)
    gravity_and_bias_compensated = get_bias_compensated_wrench(gravity_compensated, current_bias)
    return gravity_and_bias_compensated
    


if __name__ == "__main__":

    import threading
    from interfaces import FTSInterfaces
    from twisted.internet import reactor
    import roslibpy
    import time


    ### Initialize ABB-EGM deep_interfaces
    # ROS connection init using roslibpy
    ros_bridge_ip = 'localhost'     #localhost = 127.0.0.1
    ros_connection = roslibpy.Ros(host=ros_bridge_ip, port=9090)

    ros_thread = threading.Thread(target=reactor.run, args=(False,))	# Start the roslibpy reactor in a separate thread to avoid blocking main thread
    ros_thread.daemon = True
    ros_thread.start()

    # Create deep timber interfaces instance
    sync_event = threading.Event()				#event to sync the input of new egm data and the steps of the rollout
    deep_interfaces = FTSInterfaces("robot11", ros_connection, sync_event, 1.0)
    deep_interfaces.wait_ready()

    raw_wrench = deep_interfaces.ft_data__raw
    bias = deep_interfaces.current_bias

    while True:
        try:
            unbiased_wrench = get_bias_compensated_wrench(raw_wrench, bias)
            print("unbiased_wrench =", unbiased_wrench)
            time.sleep(0.1)
        except:
            pass
