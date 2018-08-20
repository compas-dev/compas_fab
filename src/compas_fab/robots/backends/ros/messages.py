from compas.geometry import Frame
from compas.robots.model.geometry import SCALE_FACTOR

from compas.geometry.transformations import basis_vectors_from_matrix
from compas.geometry.transformations import matrix_from_quaternion

__all__ = ['Pose', 'PoseStamped', 'JointState', 'MultiDOFJointState',
           'JointTrajectory', 'RobotState', 'PositionIKRequest']


class ROSmsg(object):
    """The base class for ros messages.
    """

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

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
        return cls(**msg)

    def __str__(self):
        return str(self.msg)

# ------------------------------------------------------------------------------
# std_msgs
# ------------------------------------------------------------------------------


class Time(ROSmsg):
    def __init__(self, secs=0., nsecs=0.):
        self.secs = secs
        self.nsecs = nsecs


class Header(ROSmsg):
    """http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    def __init__(self, seq=0, stamp=Time(), frame_id='/world'):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id

# ------------------------------------------------------------------------------
# geometry_msgs
# ------------------------------------------------------------------------------


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
        point = [msg['position']['x'] * SCALE_FACTOR,
                 msg['position']['y'] * SCALE_FACTOR,
                 msg['position']['z'] * SCALE_FACTOR]
        quaternion = [msg['orientation']['w'], msg['orientation']['x'],
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
        pose['position'] = {'x': self.point[0]/SCALE_FACTOR,
                            'y': self.point[1]/SCALE_FACTOR,
                            'z': self.point[2]/SCALE_FACTOR}
        qw, qx, qy, qz = self.quaternion
        pose['orientation'] = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
        return pose


class PoseStamped(ROSmsg):
    """http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    """
    def __init__(self, header=Header(), pose=Pose.worldXY()):
        self.header = header
        self.pose = pose


# ------------------------------------------------------------------------------
# sensor_msgs
# ------------------------------------------------------------------------------


class JointState(ROSmsg):
    """http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html
    """

    def __init__(self, header=Header(), name=[], position=[], velocity=[],
                 effort=[]):
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
    """http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/MultiDOFJointState.html
    """
    def __init__(self, header=Header(), joint_names=[], transforms=[], twist=[],
                 wrench=[]):
        self.header = header
        self.joint_names = joint_names
        self.transforms = transforms
        self.twist = twist
        self.wrench = wrench

# ------------------------------------------------------------------------------
# trajectory_msgs
# ------------------------------------------------------------------------------


class JointTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html
    """
    def __init__(self, header=Header(), joint_names=[], points=[]):
        self.header = header
        self.joint_names = joint_names
        self.points = points

# ------------------------------------------------------------------------------
# object_recognition_msgs
# ------------------------------------------------------------------------------


class ObjectType(ROSmsg):
    """http://docs.ros.org/kinetic/api/object_recognition_msgs/html/msg/ObjectType.html
    """
    def __init__(self, key="key", db="db"):
        self.key = key
        self.db = db

# ------------------------------------------------------------------------------
# shape_msgs
# ------------------------------------------------------------------------------


class SolidPrimitive(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/SolidPrimitive.html
    """
    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4
    BOX_X = 0
    BOX_Y = 1
    BOX_Z = 2
    SPHERE_RADIUS = 0
    CYLINDER_HEIGHT = 0
    CYLINDER_RADIUS = 1
    CONE_HEIGHT = 0
    CONE_RADIUS = 1

    def __init__(self, type=1, dimensions=[1, 1, 1]):
        self.type = type
        self.dimensions = dimensions


class Mesh(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Mesh.html
    """
    def __init__(self, triangles=[], vertices=[]):
        self.triangles = triangles
        self.vertices = vertices


class Plane(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Plane.html
    """
    def __init__(self, coef):
        self.coef = coef

# ------------------------------------------------------------------------------
# moveit_msgs
# ------------------------------------------------------------------------------


class CollisionObject(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/CollisionObject.html
    """
    ADD = 0
    REMOVE = 1
    APPEND = 2
    MOVE = 3

    def __init__(self, header=Header(), id="collision_obj", type=ObjectType(),
                 primitives=[], primitive_poses=[], meshes=[], mesh_poses=[],
                 planes=[], plane_poses=[], operation=0):
        self.header = header
        self.id = id
        self.type = type
        self.primitives = primitives
        self.primitive_poses = primitive_poses
        self.meshes = meshes
        self.mesh_poses = mesh_poses
        self.planes = planes
        self.plane_poses = plane_poses
        self.operation = operation  # ADD or REMOVE or APPEND or MOVE


class AttachedCollisionObject(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html
    """

    def __init__(self, link_name='ee_link', object=CollisionObject(),
                 touch_links=[], detach_posture=JointTrajectory(), weight=0):
        self.link_name = link_name
        self.object = object
        self.touch_links = touch_links
        self.detach_posture = detach_posture
        self.weight = weight


class Constraints(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/Constraints.html
    """
    def __init__(self, name='', joint_constraints=[], position_constraints=[],
                 orientation_constraints=[], visibility_constraints=[]):
        self.name = name
        self.joint_constraints = joint_constraints
        self.position_constraints = position_constraints
        self.orientation_constraints = orientation_constraints
        self.visibility_constraints = visibility_constraints


class RobotState(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotState.html
    """
    def __init__(self, joint_state=JointState(),
                 multi_dof_joint_state=MultiDOFJointState(),
                 attached_collision_objects=[], is_diff=False):
        self.joint_state = joint_state
        self.multi_dof_joint_state = multi_dof_joint_state
        self.attached_collision_objects = attached_collision_objects
        self.is_diff = is_diff


class PositionIKRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PositionIKRequest.html
    """
    def __init__(self, group_name="robot", robot_state=RobotState(),
                 constraints=Constraints(), pose_stamped=PoseStamped(),
                 timeout=1.0, attempts=8, avoid_collisions=True):
        self.group_name = group_name
        self.robot_state = robot_state
        self.constraints = constraints
        self.avoid_collisions = avoid_collisions
        self.pose_stamped = pose_stamped
        self.timeout = timeout
        self.attempts = attempts


if __name__ == '__main__':

    header = Header(frame_id='base_link')
    pose = Pose([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    pose_stamped = PoseStamped(header=header, pose=pose)
    print(pose_stamped, "\n")

    group_name = 'manipulator'
    ik_request = PositionIKRequest(group_name=group_name, pose_stamped=pose_stamped)
    print(ik_request, "\n")

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_positions = [6.254248742364907, -0.06779616254839081, 4.497665741209763, -4.429869574230193, -4.741325546996638, 3.1415926363120015]

    joint_state = JointState(name=joint_names, position=joint_positions)
    print(joint_state, "\n")
