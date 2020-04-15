from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.backends.ik_solver import fit_within_bounds


class InverseKinematicsSolver(object):
    """Create a custom InverseKinematicsSolver for a robot.
    """
    # TODO create with class `Tool`, not tool frame!! 
    def __init__(self, robot, group, function, base_frame=None, tool_frame=None):

        self.robot = robot
        self.group = group
        self.joints = robot.get_configurable_joints(group=group)
        self.joint_names = robot.get_configurable_joint_names(group=group)
        self.function = function

        self.base_transformation = Transformation.from_frame(base_frame).inverse() if base_frame else None
        self.tool_transformation = Transformation.from_frame(tool_frame).inverse() if tool_frame else None

    def update_base_transformation(self, base_frame):
        self.base_transformation = Transformation.from_frame(base_frame).inverse()
    
    def convert_frame_wcf_to_tool0_wcf(self, frame_wcf):
        return Frame.from_transformation(Transformation.from_frame(frame_wcf) * self.tool_transformation)

    def convert_frame_wcf_to_frame_tool0_rcf(self, frame_wcf, base_transformation=None, tool_transformation=None):
        T = Transformation.from_frame(frame_wcf)
        if base_transformation:
            T = base_transformation * T
        if tool_transformation:
            T = T * tool_transformation
        return Frame.from_transformation(T)
        # return Frame.from_transformation(self.base_transformation * Transformation.from_frame(frame_wcf) * self.tool_transformation)

    def try_to_fit_configurations_between_bounds(self, configurations):
        """
        """
        j1, j2, j3, j4, j5, j6 = self.joints
        for i, c in enumerate(configurations):
            a1, a2, a3, a4, a5, a6 = c.values
            try:
                a1 = fit_within_bounds(a1, j1.limit.lower, j1.limit.upper)
                a2 = fit_within_bounds(a2, j2.limit.lower, j2.limit.upper)
                a3 = fit_within_bounds(a3, j3.limit.lower, j3.limit.upper)
                a4 = fit_within_bounds(a4, j4.limit.lower, j4.limit.upper)
                a5 = fit_within_bounds(a5, j5.limit.lower, j5.limit.upper)
                a6 = fit_within_bounds(a6, j6.limit.lower, j6.limit.upper)
                configurations[i].values = [a1, a2, a3, a4, a5, a6]
            except AssertionError:
                configurations[i] = None
        return configurations

    def add_joint_names_to_configurations(self, configurations):
        """
        """
        for i, c in enumerate(configurations):
            configurations[i].joint_names = self.joint_names[:]

    def inverse_kinematics_function(self):
        """
        """

        def inverse_kinematics(frame_WCF, start_configuration=None, group=None,
                               avoid_collisions=True, constraints=None,
                               attempts=8, attached_collision_meshes=None,
                               return_full_configuration=False,
                               cull=False,
                               return_closest_to_start=False,
                               return_idxs=None):

            if start_configuration:
                base_frame = self.robot.get_base_frame(self.group, full_configuration=start_configuration)
                self.update_base_transformation(base_frame)
                # in_collision = self.robot.client.configuration_in_collision(start_configuration)
                # if in_collision:
                #    raise ValueError("Start configuration already in collision")

            # frame_tool0_RCF = self.convert_frame_wcf_to_frame_tool0_rcf(frame_WCF)
            frame_tool0_RCF = Frame.from_transformation(self.base_transformation * Transformation.from_frame(frame_WCF) * self.tool_transformation)

            # call the ik function
            configurations = self.function(frame_tool0_RCF)

            # The ik solution for 6 axes industrial robots returns by default 8
            # configurations, which are sorted. That means, the if you call ik
            # on 2 frames that are close to each other, and compare the 8
            # configurations of the first one with the 8 of the second one at
            # their respective indices, then these configurations are 'close' to
            # each other. That is why for certain use cases, e.g. custom cartesian
            # path planning it makes sense to keep the sorting and set the ones
            # that are out of joint limits or in collison to `None`.

            if return_idxs:
                configurations = [configurations[i] for i in return_idxs]

            # add joint names to configurations
            self.add_joint_names_to_configurations(configurations)
            
            # fit configurations within joint bounds (sets those to `None` that are not working)
            self.try_to_fit_configurations_between_bounds(configurations)
            # check collisions for all configurations (sets those to `None` that are not working)
            if self.robot.client:
                self.robot.client.check_configurations_for_collision(configurations)
            
            if return_closest_to_start:
                diffs = [c.max_difference(start_configuration) for c in configurations if c is not None]
                if len(diffs):
                    idx = diffs.index(min(diffs))
                    return configurations[idx] # only one
                return None

            if cull:
                configurations = [c for c in configurations if c is not None]

            
            return configurations
        return inverse_kinematics
