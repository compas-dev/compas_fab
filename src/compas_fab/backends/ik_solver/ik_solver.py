from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.backends.ik_solver import fit_within_bounds
from compas_fab.backends.interfaces import InverseKinematics

"""
class InverseKinematics(object):
    def __call__(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        return self.inverse_kinematics(robot, frame_WCF, start_configuration, group, options)

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
"""


class AnalyticalInverseKinematics(object):

    def __init__(self, robot, start_configuration=None, group=None):
        self.robot = robot
        self.group = group or robot.main_group_name
        self.start_configuration = start_configuration

    def __call__(self, frame_WCF, options=None):
        return self.inverse_kinematics(frame_WCF, options)

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, return_full_configuration=False, options=None):

        start_configuration = start_configuration or self.start_configuration
        group = group or self.group

        frame = frame_WCF
        if self.robot.attached_tool:
            frame = self.robot.attached_tool.from_tcf_to_t0cf([frame])[0]

        if start_configuration:
            base_frame = self.robot.get_base_frame(group, full_configuration=start_configuration)
            frame = base_frame.to_local_coordinates(frame)

        return self.__inverse_kinematics(frame)

    def __inverse_kinematics(self, frame):
        pass

        return UR5_Kinematics().inverse(frame)


class InverseKinematicsSolver(InverseKinematics):
    """Create a custom InverseKinematicsSolver for a robot.


    Examples
    --------

    >>> ik_solver = InverseKinematicsSolver(robot, "robotA", ik_abb_irb4600_40_255, base_frame, robotA_tool.frame)
    >>> robot.inverse_kinematics = ik_solver.inverse_kinematics_function()
    >>> ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    >>> ik_solver = InverseKinematicsSolver(robot, "robotA", ikfast_fn, base_frame, robotA_tool.frame)
    """
    # TODO create with class `Tool`, not tool frame!!

    def __init__(self, robot, start_configuration=None, group=None):

        if robot.attached_tool:
            tool_frame = robot.tool.tool_model.frame

        base_frame = robot.get_base_frame(group=group, full_configuration=start_configuration)

        self.base_transformation = Transformation.from_frame(base_frame).inverse() if base_frame else None
        self.tool_transformation = Transformation.from_frame(tool_frame).inverse() if tool_frame else None

    def convert_frame_wcf_to_frame_tool0_rcf(self, frame_wcf, base_transformation=None, tool_transformation=None):
        frame = frame_wcf
        if self.robot.attached_tool:
            frame = self.robot.attached_tool.from_tcf_to_t0cf([frame_wcf])[0]
        if self.base_frame:
            frame = self.base_frame.to_local_coordinates(frame)
        return frame

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

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
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
                    return configurations[idx]  # only one
                return None

            if cull:
                configurations = [c for c in configurations if c is not None]

            return configurations
        return inverse_kinematics


class UR5KinematicsSolver(AnalyticalInverseKinematics):

    def __inverse_kinematics(self, frame):
        return UR5_Kinematics().inverse(frame)


if __name__ == "__main__":
    import compas_fab
    from compas_fab.robots.ur5 import Robot
    from compas_fab.backends.ik_solver import UR5_Kinematics
    from compas_fab.robots import Tool
    from compas.datastructures import Mesh
    from compas.geometry import Point, Vector

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    robot = Robot(load_geometry=False)
    robot.attach_tool(Tool(mesh, Frame((0.07, 0, 0), (0, 0, 1), (0, 1, 0)), name="light_pen"))

    start_configuration = robot.zero_configuration()
    kinematics = UR5KinematicsSolver(robot, start_configuration=start_configuration)

    f = Frame((0.5, 0.0, 0.3), (0.1, -0.1, -1.0), (-0.0, -1.0, 0.1))
    configurations = kinematics.inverse_kinematics(f)
    for c in configurations:
        print(c)
