from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.ros.messages import Header, Pose, PoseStamped, JointState, RobotState, MultiDOFJointState, AttachedCollisionObject, PositionIKRequest, GetPositionIKRequest, \
    GetPositionIKResponse
from compas_fab.backends.ros.backend_features.helpers import validate_response, _convert_constraints_to_rosmsg
from compas_fab.backends.ros.planner_backend import ServiceDescription


class MoveItInverseKinematics(object):
    GET_POSITION_IK = ServiceDescription('/compute_ik',
                                         'GetPositionIK',
                                         GetPositionIKRequest,
                                         GetPositionIKResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def __call__(self, robot, frame, group,
                 start_configuration, avoid_collisions=True,
                 constraints=None, attempts=8,
                 attached_collision_meshes=None):

        return self.inverse_kinematics(robot, frame, group,
                                       start_configuration, avoid_collisions,
                                       constraints, attempts,
                                       attached_collision_meshes)

    def inverse_kinematics(self, robot, frame, group,
                           start_configuration, avoid_collisions=True,
                           constraints=None, attempts=8,
                           attached_collision_meshes=None):
        kwargs = {}
        kwargs['robot'] = robot
        kwargs['frame'] = frame
        kwargs['group'] = group
        kwargs['start_configuration'] = start_configuration
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['constraints'] = constraints
        kwargs['attempts'] = attempts
        # why is attached_collision_meshes unused?

        kwargs['errback_name'] = 'errback'

        return await_callback(self.inverse_kinematics_async, **kwargs)

    def inverse_kinematics_async(self, callback, errback, robot, frame, group,
                                 start_configuration, avoid_collisions=True,
                                 constraints=None, attempts=8, attached_collision_meshes=None):
        """Asynchronous handler of MoveIt IK service."""
        base_link = robot.model.root.name
        header = Header(frame_id=base_link)
        pose = Pose.from_frame(frame)
        pose_stamped = PoseStamped(header, pose)
        joint_state = JointState(
            name=start_configuration.joint_names, position=start_configuration.values, header=header)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        constraints = _convert_constraints_to_rosmsg(constraints, header)

        ik_request = PositionIKRequest(group_name=group,
                                       robot_state=start_state,
                                       constraints=constraints,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=avoid_collisions,
                                       attempts=attempts)

        def convert_to_positions(response):
            callback((response.solution.joint_state.position, response.solution.joint_state.name))

        self.GET_POSITION_IK(self.ros_client, (ik_request, ), convert_to_positions, errback)
