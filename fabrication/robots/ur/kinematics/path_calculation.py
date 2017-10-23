import math
from compas_fabrication.fabrication.utilities import sign
from compas_fabrication.fabrication.robots.ur.kinematics import inverse_kinematics

def format_joint_positions(joint_positions_a, joint_positions_b = [0,0,0,0,0,0]):
    """Add or subtract 2*pi to the joint positions a, so that they have the 
    least difference to joint positions b.
    """
     
    new_joint_positions = []
    for ja_1, jb in zip(joint_positions_a, joint_positions_b):
        ja_2 = ja_1 - math.pi * 2 * sign(ja_1)
        if math.fabs(jb - ja_1) < math.fabs(jb - ja_2):
            new_joint_positions.append(ja_1)
        else:
            new_joint_positions.append(ja_2)
    return new_joint_positions


def calculate_configurations_for_path(frames, ur_params, current_positions = []):
    """Calculate possible configurations for a path.
    
    Args:
        frames (Frame): the path described with frames
    
    Returns:
        configurations: list of list of float
    """
        
    configurations = []
    
    for i, frame in enumerate(frames):
        qsols = inverse_kinematics(frame, ur_params)
        if not len(qsols):
            return []
        if i == 0:
            if len(current_positions):
                qsols_formatted = []
                for jp_a in qsols:
                    jp_a_formatted = format_joint_positions(jp_a, current_positions)
                    qsols_formatted.append(jp_a_formatted)
                configurations.append(qsols_formatted)
            else:  
                configurations.append(qsols)
        else:
            previous_qsols = configurations[-1][:]
            qsols_sorted = []  
            for jp_b in previous_qsols:
                diffs = []
                qsols_formatted = []
                for jp_a in qsols:
                    jp_a_formatted = format_joint_positions(jp_a, jp_b)
                    qsols_formatted.append(jp_a_formatted)
                    diffs.append(sum([math.fabs(qa - qb) for qa, qb in zip(jp_a_formatted, jp_b)]))
                selected_idx = diffs.index(min(diffs))
                qsols_sorted.append(qsols_formatted[selected_idx])
            configurations.append(qsols_sorted)
                   
    return zip(*configurations)


