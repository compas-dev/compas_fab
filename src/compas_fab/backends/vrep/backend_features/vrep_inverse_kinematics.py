from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.backend_feature_interfaces import InverseKinematics
from compas_fab.backends.vrep.helpers import config_from_vrep
from compas_fab.backends.vrep.helpers import frame_to_vrep_pose


class VrepInverseKinematics(InverseKinematics):
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options={}):
        metric_values = options.get('metric_values')
        gantry_joint_limits = options.get('gantry_joint_limits')
        arm_joint_limits = options.get('arm_joint_limits')
        max_trials = options.get('max_trials')
        max_results = options.get('max_results', 1)
        num_joints = options['num_joints']
        return self.inverse_kinematics_deprecated(group, frame_WCF, num_joints, metric_values, gantry_joint_limits, arm_joint_limits, max_trials, max_results)

    def inverse_kinematics_deprecated(self, group, goal_frame, num_joints, metric_values=None, gantry_joint_limits=None, arm_joint_limits=None, max_trials=None, max_results=1):
        """Calculates inverse kinematics to find valid robot configurations for the specified goal frame.

        Args:
            group (:obj:`int`): Integer referencing the desired robot group.
            goal_frame (:class:`Frame`): Target or goal frame.
            num_joints (:obj:`int`): Number of configurable joints
            metric_values (:obj:`list` of :obj:`float`): List containing one value
                per configurable joint. Each value ranges from 0 to 1,
                where 1 indicates the axis/joint is blocked and cannot
                move during inverse kinematic solving.
            gantry_joint_limits (:obj:`list` of `float`): List of 6 floats defining the upper/lower limits of
                gantry joints. Use this if you want to restrict the area in which to search for states.
            arm_joint_limits (:obj:`list` of `float`): List of 12 floats defining the upper/lower limits of
                arm joints. Use this if you want to restrict the working area in which to search for states.
            max_trials (:obj:`int`): Number of trials to run. Set to ``None``
                to retry infinitely.
            max_results (:obj:`int`): Maximum number of result states to return.

        Returns:
            list: List of :class:`Configuration` objects representing
            the collision-free configuration for the ``goal_frame``.
        """

        # joints = len(robot.get_configurable_joints())
        if not metric_values:
            metric_values = [0.1] * num_joints

        self.client.set_robot_metric(group, metric_values)

        states = self.client.find_raw_robot_states(group, frame_to_vrep_pose(goal_frame, self.client.scale), gantry_joint_limits, arm_joint_limits, max_trials, max_results)

        return [config_from_vrep(states[i:i + num_joints], self.client.scale)
                for i in range(0, len(states), num_joints)]
