from __future__ import absolute_import

from compas_fab.backends.ros.messages.geometry_msgs import Point
from compas_fab.backends.ros.messages.geometry_msgs import Pose
from compas_fab.backends.ros.messages.geometry_msgs import PoseStamped
from compas_fab.backends.ros.messages.geometry_msgs import Quaternion
from compas_fab.backends.ros.messages.geometry_msgs import Vector3
from compas_fab.backends.ros.messages.object_recognition_msgs import ObjectType
from compas_fab.backends.ros.messages.octomap_msgs import OctomapWithPose
from compas_fab.backends.ros.messages.sensor_msgs import JointState
from compas_fab.backends.ros.messages.sensor_msgs import MultiDOFJointState
from compas_fab.backends.ros.messages.shape_msgs import Mesh
from compas_fab.backends.ros.messages.shape_msgs import Plane
from compas_fab.backends.ros.messages.shape_msgs import SolidPrimitive
from compas_fab.backends.ros.messages.std_msgs import Header
from compas_fab.backends.ros.messages.std_msgs import ROSmsg
from compas_fab.backends.ros.messages.trajectory_msgs import JointTrajectory
from compas_fab.backends.ros.messages.trajectory_msgs import MultiDOFJointTrajectory


class CollisionObject(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/CollisionObject.html
    """
    ADD = 0
    REMOVE = 1
    APPEND = 2
    MOVE = 3

    def __init__(self, header=None, id="collision_obj", type=None,
                 primitives=None, primitive_poses=None, meshes=None, mesh_poses=None,
                 planes=None, plane_poses=None,
                 subframe_names=None, subframe_poses=None, operation=0):
        self.header = header or Header()  # a header, used for interpreting the poses
        self.id = id  # the id of the object (name used in MoveIt)
        self.type = type or ObjectType()  # The object type in a database of known objects
        # solid geometric primitives
        self.primitives = primitives or []
        self.primitive_poses = primitive_poses or []
        # meshes
        self.meshes = meshes or []
        self.mesh_poses = mesh_poses or []
        # bounding planes
        self.planes = planes or []
        self.plane_poses = plane_poses or []

        self.operation = operation  # ADD or REMOVE or APPEND or MOVE

    @classmethod
    def from_collision_mesh(cls, collision_mesh):
        """Creates a collision object from a :class:`compas_fab.robots.CollisionMesh`
        """
        kwargs = {}
        kwargs['header'] = Header(frame_id=collision_mesh.root_name)
        kwargs['id'] = collision_mesh.id
        kwargs['meshes'] = [Mesh.from_mesh(collision_mesh.mesh)]
        kwargs['mesh_poses'] = [Pose.from_frame(collision_mesh.frame)]

        return cls(**kwargs)

    @classmethod
    def from_msg(cls, msg):
        kwargs = {}

        kwargs['header'] = Header.from_msg(msg['header'])
        kwargs['id'] = msg['id']
        kwargs['type'] = ObjectType.from_msg(msg['type'])

        kwargs['primitives'] = [SolidPrimitive.from_msg(i) for i in msg['primitives']]
        kwargs['primitive_poses'] = [Pose.from_msg(i) for i in msg['primitive_poses']]
        kwargs['meshes'] = [Mesh.from_msg(i) for i in msg['meshes']]
        kwargs['mesh_poses'] = [Pose.from_msg(i) for i in msg['mesh_poses']]
        kwargs['planes'] = [Plane.from_msg(i) for i in msg['planes']]
        kwargs['plane_poses'] = [Pose.from_frame(i) for i in msg['plane_poses']]

        kwargs['operation'] = msg['operation']

        return cls(**kwargs)


class AttachedCollisionObject(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/AttachedCollisionObject.html
    """

    def __init__(self, link_name=None, object=None, touch_links=None,
                 detach_posture=None, weight=0):
        self.link_name = link_name or ''
        self.object = object or CollisionObject()
        self.touch_links = touch_links or []
        self.detach_posture = detach_posture or JointTrajectory()
        self.weight = weight

    @classmethod
    def from_attached_collision_mesh(cls, attached_collision_mesh):
        """Creates an attached collision object from a :class:`compas_fab.robots.AttachedCollisionMesh`
        """

        kwargs = {}
        kwargs['link_name'] = attached_collision_mesh.link_name
        kwargs['object'] = CollisionObject.from_collision_mesh(attached_collision_mesh.collision_mesh)
        kwargs['touch_links'] = [str(s) for s in attached_collision_mesh.touch_links]
        kwargs['weight'] = attached_collision_mesh.weight

        return cls(**kwargs)


class Constraints(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/Constraints.html
    """

    def __init__(self, name='', joint_constraints=None, position_constraints=None,
                 orientation_constraints=None, visibility_constraints=None):
        self.name = name
        self.joint_constraints = joint_constraints if joint_constraints else []
        self.position_constraints = position_constraints if position_constraints else []
        self.orientation_constraints = orientation_constraints if orientation_constraints else []
        self.visibility_constraints = visibility_constraints if visibility_constraints else []


class RobotState(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotState.html
    """

    def __init__(self, joint_state=None, multi_dof_joint_state=None,
                 attached_collision_objects=None, is_diff=False):
        self.joint_state = joint_state if joint_state else JointState()
        self.multi_dof_joint_state = multi_dof_joint_state if multi_dof_joint_state else MultiDOFJointState()
        self.attached_collision_objects = attached_collision_objects if attached_collision_objects else []
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
    """

    def __init__(self, group_name="robot", robot_state=None, constraints=None,
                 pose_stamped=None, timeout=1.0, attempts=8,
                 avoid_collisions=True):
        self.group_name = group_name
        self.robot_state = robot_state if robot_state else RobotState()
        self.constraints = constraints if constraints else Constraints()
        self.avoid_collisions = avoid_collisions
        self.pose_stamped = pose_stamped if pose_stamped else PoseStamped()
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

    def __int__(self):
        return self.val

    def __eq__(self, other):
        return self.val == other

    def __ne__(self, other):
        return self.val != other

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

    def __init__(self, keys=None, values=None, descriptions=None):
        self.keys = keys or []                  # parameter names (same size as values)
        self.values = values or []              # parameter values (same size as keys)
        self.descriptions = descriptions or []  # parameter description (can be empty)


class WorkspaceParameters(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/WorkspaceParameters.html
    """

    def __init__(self, header=None, min_corner=None, max_corner=None):
        self.header = header or Header()
        self.min_corner = min_corner or Vector3(-1000, -1000, -1000)
        self.max_corner = max_corner or Vector3(1000, 1000, 1000)


class TrajectoryConstraints(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/TrajectoryConstraints.html
    """

    def __init__(self, constraints=None):
        self.constraints = constraints or []  # Constraints[]


class JointConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/JointConstraint.html
    """

    def __init__(self, joint_name="", position=0, tolerance_above=0, tolerance_below=0, weight=1.):
        self.joint_name = joint_name
        self.position = float(position)
        self.tolerance_above = float(tolerance_above)
        self.tolerance_below = float(tolerance_below)
        self.weight = float(weight)

    @classmethod
    def from_joint_constraint(cls, joint_constraint):
        """Creates a `JointConstraint` from a :class:`compas_fab.robots.JointConstraint`.
        """
        c = joint_constraint
        return cls(c.joint_name, c.value, c.tolerance_above, c.tolerance_below, c.weight)


class VisibilityConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/VisibilityConstraint.html
    """

    def __init__(self):
        raise NotImplementedError


class BoundingVolume(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/BoundingVolume.html
    """

    def __init__(self, primitives=None, primitive_poses=None, meshes=None,
                 mesh_poses=None):
        self.primitives = primitives or []            # shape_msgs/SolidPrimitive[]
        self.primitive_poses = primitive_poses or []  # geometry_msgs/Pose[]
        self.meshes = meshes or []                    # shape_msgs/Mesh[]
        self.mesh_poses = mesh_poses or []            # geometry_msgs/Pose[]

    @classmethod
    def from_box(cls, box):
        """Creates a `BoundingVolume` from a :class:`compas.geometry.Box`.

        Parameters
        ----------
        box: `compas.geometry.Box`
        """
        primitive = SolidPrimitive.from_box(box)
        pose = Pose.from_frame(box.frame)
        return cls(primitives=[primitive], primitive_poses=[pose])

    @classmethod
    def from_sphere(cls, sphere):
        """Creates a `BoundingVolume` from a :class:`compas.geometry.Sphere`.

        Parameters
        ----------
        sphere: `compas.geometry.Sphere`
        """
        primitive = SolidPrimitive.from_sphere(sphere)
        pose = Pose(Point(*sphere.point), Quaternion(0, 0, 0, 1))
        return cls(primitives=[primitive], primitive_poses=[pose])

    @classmethod
    def from_mesh(cls, mesh):
        """Creates a `BoundingVolume` from a :class:`compas.datastructures.Mesh`.

        Parameters
        ----------
        sphere: `compas.datastructures.Mesh`
        """
        mesh = Mesh.from_mesh(mesh)
        pose = Pose()
        return cls(meshes=[mesh], mesh_poses=[pose])

    @classmethod
    def from_bounding_volume(cls, bounding_volume):
        """Creates a `BoundingVolume` from a :class:`compas_fab.robots.BoundingVolume`.

        Parameters
        ----------
        bounding_volume: `compas_fab.robots.BoundingVolume`
        """
        if bounding_volume.type == bounding_volume.BOX:
            return cls.from_box(bounding_volume.volume)
        elif bounding_volume.type == bounding_volume.SPHERE:
            return cls.from_sphere(bounding_volume.volume)
        elif bounding_volume.type == bounding_volume.MESH:
            return cls.from_mesh(bounding_volume.volume)
        else:
            raise NotImplementedError


class PositionConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PositionConstraint.html
    """

    def __init__(self, header=None, link_name=None, target_point_offset=None,
                 constraint_region=None, weight=None):
        self.header = header or Header()
        self.link_name = link_name or ""
        self.target_point_offset = target_point_offset or Vector3(0., 0., 0.)  # geometry_msgs/Vector3
        self.constraint_region = constraint_region or BoundingVolume()         # moveit_msgs/BoundingVolume
        self.weight = float(weight) or 1.

    @classmethod
    def from_position_constraint(cls, header, position_constraint):
        """Creates a `PositionConstraint` from a :class:`compas_fab.robots.PositionConstraint`.
        """
        constraint_region = BoundingVolume.from_bounding_volume(position_constraint.bounding_volume)
        return cls(header, position_constraint.link_name, None, constraint_region, position_constraint.weight)


class OrientationConstraint(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/OrientationConstraint.html
    """

    def __init__(self, header=None, orientation=None, link_name=None,
                 absolute_x_axis_tolerance=0.0, absolute_y_axis_tolerance=0.0,
                 absolute_z_axis_tolerance=0.0, weight=1):
        """
        Notes
        -----
        The naming of the absolute_x/y/z_axis_tolerances might be misleading:
        If you specify the absolute_x/y/z_axis_tolerances with [0.01, 0.01, 6.3],
        it means that the frame's x-axis and y-axis are allowed to rotate about
        the z-axis by an angle of 6.3 radians, whereas the z-axis can only change
        by 0.01.
        """
        self.header = header or Header()
        self.orientation = orientation or Quaternion()  # geometry_msgs/Quaternion
        self.link_name = link_name or ""
        self.absolute_x_axis_tolerance = float(absolute_x_axis_tolerance)
        self.absolute_y_axis_tolerance = float(absolute_y_axis_tolerance)
        self.absolute_z_axis_tolerance = float(absolute_z_axis_tolerance)
        self.weight = float(weight)

    @classmethod
    def from_orientation_constraint(cls, header, orientation_constraint):
        """Creates a ``OrientationConstraint`` from a :class:`compas_fab.robots.OrientationConstraint`.
        """
        qw, qx, qy, qz = orientation_constraint.quaternion
        ax, ay, az = orientation_constraint.tolerances

        kwargs = {}
        kwargs['header'] = header
        kwargs['orientation'] = Quaternion(qx, qy, qz, qw)
        kwargs['link_name'] = orientation_constraint.link_name
        kwargs['absolute_x_axis_tolerance'] = ax
        kwargs['absolute_y_axis_tolerance'] = ay
        kwargs['absolute_z_axis_tolerance'] = az
        kwargs['weight'] = orientation_constraint.weight

        return cls(**kwargs)


class PlanningSceneComponents(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/PlanningSceneComponents.html
    """
    SCENE_SETTINGS = 1
    ROBOT_STATE = 2
    ROBOT_STATE_ATTACHED_OBJECTS = 4
    WORLD_OBJECT_NAMES = 8
    WORLD_OBJECT_GEOMETRY = 16
    OCTOMAP = 32
    TRANSFORMS = 64
    ALLOWED_COLLISION_MATRIX = 128
    LINK_PADDING_AND_SCALING = 256
    OBJECT_COLORS = 512

    def __init__(self, components=None):
        self.components = components or self.SCENE_SETTINGS

    def __eq__(self, other):
        return self.components == other

    @property
    def human_readable(self):
        cls = type(self)
        for k, v in cls.__dict__.items():
            if v == self.components:
                return k
        return ''


class AllowedCollisionMatrix(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/msg/AllowedCollisionMatrix.html
    """

    def __init__(self, entry_names=None, entry_values=None, default_entry_names=None, default_entry_values=None):
        self.entry_names = entry_names or []  # string[]
        self.entry_values = entry_values or []  # moveit_msgs/AllowedCollisionEntry[]
        self.default_entry_names = default_entry_names or []  # string[]
        self.default_entry_values = default_entry_values or []  # bool[]


class PlanningSceneWorld(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/msg/PlanningSceneWorld.html
    """

    def __init__(self, collision_objects=None, octomap=None):
        self.collision_objects = collision_objects or []  # collision objects # CollisionObject[]
        self.octomap = octomap or OctomapWithPose()  # octomap_msgs/OctomapWithPose

    @classmethod
    def from_msg(cls, msg):
        collision_objects = [CollisionObject.from_msg(i) for i in msg['collision_objects']]
        octomap = msg['octomap']  # TODO: Add OctomapWithPose.from_msg(msg['octomap'])

        return cls(collision_objects, octomap)


class PlanningScene(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/msg/PlanningScene.html
    """

    def __init__(self, name='', robot_state=None, robot_model_name='',
                 fixed_frame_transforms=None, allowed_collision_matrix=None,
                 link_padding=None, link_scale=None, object_colors=None, world=None,
                 is_diff=False):
        self.name = name                                            # string
        self.robot_state = robot_state or RobotState()              # moveit_msgs/RobotState
        self.robot_model_name = robot_model_name                    # string
        self.fixed_frame_transforms = fixed_frame_transforms or []  # geometry_msgs/TransformStamped[]
        self.allowed_collision_matrix = allowed_collision_matrix or AllowedCollisionMatrix()
        self.link_padding = link_padding or []                      # moveit_msgs/LinkPadding[]
        self.link_scale = link_scale or []                          # moveit_msgs/LinkScale[]
        self.object_colors = object_colors or []                    # moveit_msgs/ObjectColor[]
        self.world = world or PlanningSceneWorld()                  # moveit_msgs/PlanningSceneWorld
        self.is_diff = is_diff                                      # bool

    @classmethod
    def from_msg(cls, msg):
        robot_state = RobotState.from_msg(msg['robot_state'])
        allowed_collision_matrix = msg['allowed_collision_matrix']
        world = PlanningSceneWorld.from_msg(msg['world'])

        return cls(msg['name'], robot_state, msg['robot_model_name'],
                   msg['fixed_frame_transforms'], allowed_collision_matrix,
                   msg['link_padding'], msg['link_scale'], msg['object_colors'],
                   world, msg['is_diff'])


class ExecuteTrajectoryGoal(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/action/ExecuteTrajectory.html
    """

    def __init__(self, trajectory=None):
        self.trajectory = trajectory or RobotTrajectory()


class ExecuteTrajectoryFeedback(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/action/ExecuteTrajectory.html
    """

    def __init__(self, state=None):
        self.state = state

    @classmethod
    def from_msg(cls, msg):
        return cls(msg['state'])


class ExecuteTrajectoryResult(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/action/ExecuteTrajectory.html
    """

    def __init__(self, error_code=None):
        self.error_code = error_code or MoveItErrorCodes()  # moveit_msgs/MoveItErrorCodes

    @classmethod
    def from_msg(cls, msg):
        error_code = MoveItErrorCodes.from_msg(msg['error_code'])
        return cls(error_code)
