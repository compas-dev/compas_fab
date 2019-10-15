"""
Internal implementation of the planner backend interface for MoveIt!
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from roslibpy import Topic

from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import Constraints
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import GetCartesianPathResponse
from compas_fab.backends.ros.messages import GetPlanningSceneRequest
from compas_fab.backends.ros.messages import GetPlanningSceneResponse
from compas_fab.backends.ros.messages import GetPositionFKRequest
from compas_fab.backends.ros.messages import GetPositionFKResponse
from compas_fab.backends.ros.messages import GetPositionIKRequest
from compas_fab.backends.ros.messages import GetPositionIKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointConstraint
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MotionPlanRequest
from compas_fab.backends.ros.messages import MotionPlanResponse
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import OrientationConstraint
from compas_fab.backends.ros.messages import PlanningSceneComponents
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import PoseStamped
from compas_fab.backends.ros.messages import PositionConstraint
from compas_fab.backends.ros.messages import PositionIKRequest
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.planner_backend import PlannerBackend
from compas_fab.backends.ros.planner_backend import ServiceDescription
from compas_fab.robots import Configuration
from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint


def convert_trajectory_points(points, types):
    result = []

    for pt in points:
        jtp = JointTrajectoryPoint(values=pt.positions,
                                   types=types,
                                   velocities=pt.velocities,
                                   accelerations=pt.accelerations,
                                   effort=pt.effort,
                                   time_from_start=Duration(pt.time_from_start.secs, pt.time_from_start.nsecs))

        result.append(jtp)

    return result


def validate_response(response):
    """Raise an exception if the response indicates an error condition."""
    if response.error_code != MoveItErrorCodes.SUCCESS:
        raise RosError(response.error_code.human_readable,
                       int(response.error_code))


class MoveItPlanner(PlannerBackend):
    """Implement the planner backend interface based on MoveIt!
    """
    GET_POSITION_IK = ServiceDescription('/compute_ik',
                                         'GetPositionIK',
                                         GetPositionIKRequest,
                                         GetPositionIKResponse,
                                         validate_response)
    GET_POSITION_FK = ServiceDescription('/compute_fk',
                                         'GetPositionFK',
                                         GetPositionFKRequest,
                                         GetPositionFKResponse,
                                         validate_response)
    GET_CARTESIAN_PATH = ServiceDescription('/compute_cartesian_path',
                                            'GetCartesianPath',
                                            GetCartesianPathRequest,
                                            GetCartesianPathResponse,
                                            validate_response)
    GET_MOTION_PLAN = ServiceDescription('/plan_kinematic_path',
                                         'GetMotionPlan',
                                         MotionPlanRequest,
                                         MotionPlanResponse,
                                         validate_response)
    GET_PLANNING_SCENE = ServiceDescription('/get_planning_scene',
                                            'GetPlanningScene',
                                            GetPlanningSceneRequest,
                                            GetPlanningSceneResponse)

    def init_planner(self):
        self.collision_object_topic = Topic(self, '/collision_object',
                                            'moveit_msgs/CollisionObject', queue_size=None)
        self.collision_object_topic.advertise()

        self.attached_collision_object_topic = Topic(
            self, '/attached_collision_object',
            'moveit_msgs/AttachedCollisionObject', queue_size=None)
        self.attached_collision_object_topic.advertise()

    def dispose_planner(self):
        self.collision_object_topic.unadvertise()
        self.attached_collision_object_topic.unadvertise()

    # ==========================================================================
    # planning services
    # ==========================================================================

    def inverse_kinematics_async(self, callback, errback, frame, base_link, group,
                                 joint_names, joint_positions, avoid_collisions=True,
                                 constraints=None, attempts=8, attached_collision_meshes=None):
        """Asynchronous handler of MoveIt IK service."""
        header = Header(frame_id=base_link)
        pose = Pose.from_frame(frame)
        pose_stamped = PoseStamped(header, pose)
        joint_state = JointState(
            name=joint_names, position=joint_positions, header=header)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        ik_request = PositionIKRequest(group_name=group,
                                       robot_state=start_state,
                                       constraints=constraints,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=avoid_collisions,
                                       attempts=attempts)

        self.GET_POSITION_IK(self, (ik_request, ), callback, errback)

    def forward_kinematics_async(self, callback, errback, joint_positions, base_link,
                                 group, joint_names, ee_link):
        """Asynchronous handler of MoveIt FK service."""
        header = Header(frame_id=base_link)
        fk_link_names = [ee_link]
        joint_state = JointState(
            name=joint_names, position=joint_positions, header=header)
        robot_state = RobotState(
            joint_state, MultiDOFJointState(header=header))

        self.GET_POSITION_FK(self, (header, fk_link_names,
                                    robot_state), callback, errback)

    def plan_cartesian_motion_async(self, callback, errback, frames, base_link,
                                    ee_link, group, joint_names, joint_types,
                                    start_configuration, max_step, jump_threshold,
                                    avoid_collisions, path_constraints,
                                    attached_collision_meshes):
        """Asynchronous handler of MoveIt cartesian motion planner service."""
        header = Header(frame_id=base_link)
        waypoints = [Pose.from_frame(frame) for frame in frames]
        joint_state = JointState(
            header=header, name=joint_names, position=start_configuration.values)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        request = dict(header=header,
                       start_state=start_state,
                       group_name=group,
                       link_name=ee_link,
                       waypoints=waypoints,
                       max_step=float(max_step),
                       jump_threshold=float(jump_threshold),
                       avoid_collisions=bool(avoid_collisions),
                       path_constraints=path_constraints)

        def convert_to_trajectory(response):
            trajectory = JointTrajectory()
            trajectory.source_message = response
            trajectory.fraction = response.fraction
            trajectory.joint_names = response.solution.joint_trajectory.joint_names
            trajectory.points = convert_trajectory_points(response.solution.joint_trajectory.points, joint_types)
            trajectory.start_configuration = Configuration(response.start_state.joint_state.position, start_configuration.types)

            callback(trajectory)

        self.GET_CARTESIAN_PATH(self, request, convert_to_trajectory, errback)

    def plan_motion_async(self, callback, errback, goal_constraints, base_link,
                          ee_link, group, joint_names, joint_types,
                          start_configuration, path_constraints=None,
                          trajectory_constraints=None,
                          planner_id='', num_planning_attempts=8,
                          allowed_planning_time=2.,
                          max_velocity_scaling_factor=1.,
                          max_acceleration_scaling_factor=1.,
                          attached_collision_meshes=None,
                          workspace_parameters=None):
        """Asynchronous handler of MoveIt motion planner service."""

        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?

        header = Header(frame_id=base_link)
        joint_state = JointState(
            header=header, name=joint_names, position=start_configuration.values)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        # goal constraints
        constraints = Constraints()
        for c in goal_constraints:
            if c.type == c.JOINT:
                constraints.joint_constraints.append(
                    JointConstraint.from_joint_constraint(c))
            elif c.type == c.POSITION:
                constraints.position_constraints.append(
                    PositionConstraint.from_position_constraint(header, c))
            elif c.type == c.ORIENTATION:
                constraints.orientation_constraints.append(
                    OrientationConstraint.from_orientation_constraint(header, c))
            else:
                raise NotImplementedError
        goal_constraints = [constraints]

        # path constraints
        if path_constraints:
            constraints = Constraints()
            for c in path_constraints:
                if c.type == c.JOINT:
                    constraints.joint_constraints.append(
                        JointConstraint.from_joint_constraint(c))
                elif c.type == c.POSITION:
                    constraints.position_constraints.append(
                        PositionConstraint.from_position_constraint(header, c))
                elif c.type == c.ORIENTATION:
                    constraints.orientation_constraints.append(
                        OrientationConstraint.from_orientation_constraint(header, c))
                else:
                    raise NotImplementedError
            path_constraints = constraints

        request = dict(start_state=start_state,
                       goal_constraints=goal_constraints,
                       path_constraints=path_constraints,
                       trajectory_constraints=trajectory_constraints,
                       planner_id=planner_id,
                       group_name=group,
                       num_planning_attempts=num_planning_attempts,
                       allowed_planning_time=allowed_planning_time,
                       max_velocity_scaling_factor=max_velocity_scaling_factor,
                       max_acceleration_scaling_factor=max_velocity_scaling_factor)
        # workspace_parameters=workspace_parameters

        def convert_to_trajectory(response):
            trajectory = JointTrajectory()
            trajectory.source_message = response
            trajectory.fraction = 1.
            trajectory.joint_names = response.trajectory.joint_trajectory.joint_names
            trajectory.points = convert_trajectory_points(response.trajectory.joint_trajectory.points, joint_types)
            trajectory.start_configuration = Configuration(response.trajectory_start.joint_state.position, start_configuration.types)
            trajectory.planning_time = response.planning_time

            callback(trajectory)

        self.GET_MOTION_PLAN(self, request, convert_to_trajectory, errback)

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def get_planning_scene_async(self, callback, errback):
        request = dict(components=PlanningSceneComponents(PlanningSceneComponents.SCENE_SETTINGS |
                                                          PlanningSceneComponents.ROBOT_STATE |
                                                          PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                                          PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                                          PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                                          PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                                          PlanningSceneComponents.OBJECT_COLORS))
        self.GET_PLANNING_SCENE(self, request, callback, errback)

    def add_collision_mesh(self, collision_mesh):
        """Add a collision mesh to the planning scene."""
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self._collision_object(co, CollisionObject.ADD)

    def remove_collision_mesh(self, id):
        """Remove a collision mesh from the planning scene."""
        co = CollisionObject()
        co.id = id
        self._collision_object(co, CollisionObject.REMOVE)

    def append_collision_mesh(self, collision_mesh):
        """Append a collision mesh to the planning scene."""
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self._collision_object(co, CollisionObject.APPEND)

    def _collision_object(self, collision_object, operation=CollisionObject.ADD):
        collision_object.operation = operation
        self.collision_object_topic.publish(collision_object.msg)

    def add_attached_collision_mesh(self, attached_collision_mesh):
        """Add a collision mesh attached to the robot."""
        aco = AttachedCollisionObject.from_attached_collision_mesh(
            attached_collision_mesh)
        self._attached_collision_object(aco, operation=CollisionObject.ADD)

    def remove_attached_collision_mesh(self, id):
        """Add an attached collision mesh from the robot."""
        aco = AttachedCollisionObject()
        aco.object.id = id
        return self._attached_collision_object(aco, operation=CollisionObject.REMOVE)

    def _attached_collision_object(self, attached_collision_object, operation=CollisionObject.ADD):
        attached_collision_object.object.operation = operation
        self.attached_collision_object_topic.publish(attached_collision_object.msg)
