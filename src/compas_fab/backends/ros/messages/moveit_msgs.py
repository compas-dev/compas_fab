from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header

from .geometry_msgs import PoseStamped
from .geometry_msgs import Vector3
from .geometry_msgs import Quaternion

from .sensor_msgs import JointState
from .sensor_msgs import MultiDOFJointState

from .trajectory_msgs import JointTrajectory
from .trajectory_msgs import MultiDOFJointTrajectory

from .object_recognition_msgs import ObjectType

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

    @classmethod
    def from_msg(cls, msg):
        joint_state = JointState.from_msg(msg['joint_state'])
        multi_dof_joint_state = MultiDOFJointState.from_msg(
            msg['multi_dof_joint_state'])
        attached_collision_objects = [AttachedCollisionObject.from_msg(
            item) for item in msg['attached_collision_objects']]
        return cls(joint_state, multi_dof_joint_state, attached_collision_objects, msg['is_diff'])


class PositionIKRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PositionIKRequest.html

    Examples
    --------
    >>> base_link = 'base_link'
    >>> planning_group = 'manipulator'
    >>> pose = Pose([420, -25, 459], [1, 0, 0], [0, 1, 0])
    >>> joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                       'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint']
    >>> joint_positions = [3.39, -1.47, -2.05, 0.38, -4.96, -6.28]
    >>> header = Header(frame_id='base_link')
    >>> pose_stamped = PoseStamped(header, pose)
    >>> joint_state = JointState(name=joint_names, position=joint_positions,
                                 header=header)
    >>> multi_dof_joint_state = MultiDOFJointState(header=header,
                                                   joint_names=joint_names)
    >>> start_state = RobotState(joint_state, multi_dof_joint_state)
    >>> ik_request = PositionIKRequest(group_name=planning_group,
                                       robot_state=start_state,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=True)
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


class RobotTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
    """

    def __init__(self, joint_trajectory=JointTrajectory(),
                 multi_dof_joint_trajectory=MultiDOFJointTrajectory()):
        self.joint_trajectory = joint_trajectory
        self.multi_dof_joint_trajectory = multi_dof_joint_trajectory

    @classmethod
    def from_msg(cls, msg):
        joint_trajectory = JointTrajectory.from_msg(msg['joint_trajectory'])
        multi_dof_joint_trajectory = MultiDOFJointTrajectory.from_msg(
            msg['multi_dof_joint_trajectory'])
        return cls(joint_trajectory, multi_dof_joint_trajectory)


class MoveItErrorCodes(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/MoveItErrorCodes.html
    """
    # overall behavior
    SUCCESS = 1
    FAILURE = 99999

    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7

    # planning & kinematics request errors
    START_STATE_IN_COLLISION = -10
    START_STATE_VIOLATES_PATH_CONSTRAINTS = -11

    GOAL_IN_COLLISION = -12
    GOAL_VIOLATES_PATH_CONSTRAINTS = -13
    GOAL_CONSTRAINTS_VIOLATED = -14

    INVALID_GROUP_NAME = -15
    INVALID_GOAL_CONSTRAINTS = -16
    INVALID_ROBOT_STATE = -17
    INVALID_LINK_NAME = -18
    INVALID_OBJECT_NAME = -19

    # system errors
    FRAME_TRANSFORM_FAILURE = -21
    COLLISION_CHECKING_UNAVAILABLE = -22
    ROBOT_STATE_STALE = -23
    SENSOR_INFO_STALE = -24

    # kinematics errors
    NO_IK_SOLUTION = -31

    def __init__(self, val=-31):
        self.val = val

    def __eq__(self, other):
        return self.val == other
    
    @property
    def human_readable(self):
        cls = type(self)
        for k, v in cls.__dict__.items():
            if v == self.val:
                return k
        return ''


class PlannerParams(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/msg/PlannerParams.html
    """

    def __init__(self, keys=[], values=[], descriptions=[]):
        self.keys = keys # parameter names (same size as values)
        self.values = values # parameter values (same size as keys)
        self.descriptions = descriptions # parameter description (can be empty)

class WorkspaceParameters(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/WorkspaceParameters.html
    """
    def __init__(self, header=Header(), min_corner=Vector3(-100,-100,-100), max_corner=Vector3(100,100,100)):
        self.header = header
        self.min_corner = min_corner
        self.max_corner = max_corner

class TrajectoryConstraints(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/TrajectoryConstraints.html
    """
    def __init__(self, constraints=[]):
        self.constraints = constraints #Constraints[]


class JointConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/JointConstraint.html
    """
    def __init__(self):
        raise NotImplementedError

class VisibilityConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/VisibilityConstraint.html
    """
    def __init__(self):
        raise NotImplementedError

class BoundingVolume(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/BoundingVolume.html
    """
    def __init__(self, primitives=[], primitive_poses=[], meshes=[], 
                 mesh_poses=[]):
        self.primitives = primitives #shape_msgs/SolidPrimitive[] 
        self.primitive_poses = primitive_poses #geometry_msgs/Pose[] 
        self.meshes = meshes #shape_msgs/Mesh[] 
        self.mesh_poses = mesh_poses #geometry_msgs/Pose[] 

class PositionConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PositionConstraint.html
    """
    def __init__(self, header=Header(), link_name="", 
                 target_point_offset=Vector3(0.1,0.1,0.1), 
                 constraint_region=BoundingVolume(), weight=1):
        self.header = header
        self.link_name = link_name
        self.target_point_offset = target_point_offset # geometry_msgs/Vector3 
        self.constraint_region = constraint_region # moveit_msgs/BoundingVolume 
        self.weight = weight # float64 

class OrientationConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/OrientationConstraint.html
    """
    def __init__(self, header=Header(), orientation=Quaternion(), link_name="", 
                 absolute_x_axis_tolerance=0.005, absolute_y_axis_tolerance=0.005,
                 absolute_z_axis_tolerance=0.005, weight=1):
        self.header = header
        self.orientation = orientation #geometry_msgs/Quaternion 
        self.link_name = link_name
        self.absolute_x_axis_tolerance = absolute_x_axis_tolerance
        self.absolute_y_axis_tolerance = absolute_y_axis_tolerance
        self.absolute_z_axis_tolerance = absolute_z_axis_tolerance
        self.weight = weight # float64 

"""
rostopic info /attached_collision_object
Type: moveit_msgs/AttachedCollisionObject

rostopic info /collision_object
Type: moveit_msgs/CollisionObject
"""