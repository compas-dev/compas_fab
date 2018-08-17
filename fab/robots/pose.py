from compas.geometry.objects.frame import Frame
from compas.geometry.transformations import matrix_from_quaternion
from compas.geometry.transformations import basis_vectors_from_matrix

__all__ = ['Pose', 'Header', 'PoseStamped', 'PositionIKRequest']

SCALE_FACTOR = 1000.

# TODO :rename in messages??

class Pose(Frame):
    """Represents a robot pose.
    
    In principal the ``Pose`` is a wrapper object around the frame to derive 
    rosbridge messages therefrom.

    Examples:
        >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> p1 = Pose.from_frame(f)
        >>> msg = p1.msg
        >>> p2 = Pose.from_msg(msg)
        >>> print(p1 == p2)
    """

    @classmethod
    def from_frame(cls, frame):
        return cls(frame.point, frame.xaxis, frame.yaxis)

    @property
    def frame(self):
        return Frame(self.point, self.xaxis, self.yaxis)

    @classmethod
    def from_msg(cls, msg):
        point = [msg['position']['x'] * SCALE_FACTOR, msg['position']['y'] * SCALE_FACTOR, \
            msg['position']['z'] * SCALE_FACTOR]
        quaternion = [msg['orientation']['w'], msg['orientation']['x'], \
            msg['orientation']['y'], msg['orientation']['z']]
        R = matrix_from_quaternion(quaternion)
        xaxis, yaxis = basis_vectors_from_matrix(R)
        return cls(point, xaxis, yaxis)
    
    @property
    def msg(self):
        """Returns the pose as dictionary to use with rosbridge.

        http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        """
        pose = {}
        pose['position'] = {'x':self.point[0]/SCALE_FACTOR, 'y':self.point[1]/SCALE_FACTOR, 'z':self.point[2]/SCALE_FACTOR}
        qw, qx, qy, qz = self.quaternion
        pose['orientation'] = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
        return pose
    
    @property
    def pose_quaternion(self):
        """Implemented for Kathrin.
        """
        return list(self.point) + self.quaternion


# TODO get from ros or move somewhere else?? ? AUTOMATE to and from

class ROSmsg(object):

    @property
    def msg(self):
        msg = {}
        for key, value in self.__dict__.items():
            if hasattr(value, 'msg'):
                msg[key] = value.msg
            else:
                msg[key] = value
        return msg
    
    @classmethod
    def from_msg(cls, msg):
        pass

class Header(ROSmsg):
    """http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    def __init__(self, frame_id='/world', seq=0, secs=0., nsecs=0.):
        self.frame_id = frame_id
        self.seq = seq
        self.stamp = {'secs': secs, 'nsecs': nsecs}
            
    @classmethod
    def from_msg(cls, msg):
        frame_id = msg['frame_id']
        seq = msg['seq']
        secs = msg['stamp']['secs']
        nsecs = msg['stamp']['nsecs']
        return cls(frame_id, seq, secs, nsecs)

class PoseStamped(ROSmsg):
    """http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    """
    def __init__(self, header, pose):
        self.header = header
        self.pose = pose
    
class PositionIKRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PositionIKRequest.html
    """
    def __init__(self, group_name, robot_state, pose_stamped, timeout=1.0, attempts=8, avoid_collisions=True):
        self.group_name = group_name
        self.robot_state = robot_state
        #self.constraints = constraints ?
        self.avoid_collisions = avoid_collisions
        self.pose_stamped = pose_stamped
        self.timeout = timeout
        self.attempts = attempts

class JointState(ROSmsg):

    def __init__(self, header, name, position, velocity, effort):
        self.header = header
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort
    
    @classmethod
    def from_name_and_position(cls, name, position):
        return cls(Header(), name, position, [], [])

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        name = msg['name']
        position = msg['position']
        velocity = msg['velocity']
        effort = msg['effort']
        return cls(header, name, position, velocity, effort)

class MultiDOFJointState(ROSmsg):

    def __init__(self, header, joint_names, transforms=[], twist=[], wrench=[]):
        self.header = header
        self.joint_names = joint_names
        self.transforms = transforms
        self.twist = twist
        self.wrench = wrench

class RobotState(ROSmsg):
    def __init__(self, joint_state, multi_dof_joint_state, attached_collision_objects=[], is_diff=False):
        self.joint_state = joint_state
        self.multi_dof_joint_state = multi_dof_joint_state
        self.attached_collision_objects = attached_collision_objects
        self.is_diff = is_diff



if __name__ == '__main__':

    f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    p1 = Pose.from_frame(f)
    msg = p1.msg
    print(msg)
    p2 = Pose.from_msg(msg)
    print(p1 == p2)


    header = Header('base_link')
    pose = p1
    pose_stamped = PoseStamped(header, pose)
    print(header.msg)
    print(pose_stamped.msg)
    group_name = 'manipulator'
    ik_request = PositionIKRequest(group_name, pose_stamped)
    print(ik_request.msg)


    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_positions = [6.254248742364907, -0.06779616254839081, 4.497665741209763, -4.429869574230193, -4.741325546996638, 3.1415926363120015]
    
    joint_state = JointState.from_name_and_position(joint_names, joint_positions)
    print(joint_state.msg)

    multi_dof_joint_state = MultiDOFJointState(header, joint_names)
    print(multi_dof_joint_state.msg)