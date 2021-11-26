from compas.geometry import Frame
from compas.robots import Configuration
from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.kinematics.utils import fit_within_bounds
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.kinematics.spherical_wrist_kinematics import *  # noqa: F403, F401
from compas_fab.backends.kinematics.offset_wrist_kinematics import *  # noqa: F403, F401


class AnalyticalInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

    def __init__(self, client=None):
        self.client = client

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic (IK) for a given frame.

        The IK for 6-axis industrial robots returns by default 8 possible solutions.
        These solutions have an order. That means that if you call IK on two
        subsequent frames and compare the 8 configurations of the first frame
        with the 8 configurations of the second frame at their respective indices,
        then these configurations are "close" to one another. For this reason,
        for certain use cases, e.g. for cartesian path planning, it makes sense
        to keep the order of solutions. This can be achieved by setting the
        optional parameter ``keep_order`` to ``True``. The configurations that
        are in collision or outside joint boundaries are then not removed from
        the list of solutions, they are set to ``None``.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The start_configuration of the robot.
        group: str, optional
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:
            - ``"check_collision"``: (:obj:`str`, optional ) When ``True``, checks
                if the robot is in collision. Defaults to ``False``.
            - ``"keep_order"``: (:obj:`str`, optional ) When ``False``, removes the
                ``None``- solutions. Defaults to ``False``.

        Yields
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list
            of matching joint names. If ``"keep_order"`` is ``True`` this list
            contains also ``None``, ``None``
        """
        # What is the most elegant way to do this?
        inverse_kinematics_function = eval("%sKinematics().inverse" % robot.name.upper())

        keep_order = options["keep_order"] if options and "keep_order" in options else False

        # TODO
        # what is the expected behaviour of this function?
        # - if start_configuration is passed, should we move the solution which
        #   is closest at the first place? if yes, then keeping the order makes
        #   no sense anymore

        # convert the frame WCF to RCF
        base_frame = robot.get_base_frame(group=group, full_configuration=start_configuration)
        frame_RCF = base_frame.to_local_coordinates(frame_WCF)

        # calculate inverse with 8 solutions
        solutions = inverse_kinematics_function(frame_RCF)
        configurations = self.joint_angles_to_configurations(robot, solutions, group=group)

        # check collisions for all configurations (>> sets those to `None` that are not working)
        if options and "check_collision" in options and options["check_collision"] is True:
            for i, config in enumerate(configurations):
                try:
                    self.client.check_collisions(robot, config)
                except BackendError:
                    configurations[i] = None

        # fit configurations within joint bounds (>> sets those to `None` that are not working)
        configurations = self.try_to_fit_configurations_between_bounds(robot, configurations, group=group)

        for config in configurations:
            if config:
                yield config.joint_values, config.joint_names
            elif keep_order:
                yield None, None

    def joint_angles_to_configurations(self, robot, solutions, group=None):
        joint_names = robot.get_configurable_joint_names(group=group)
        return [Configuration.from_revolute_values(q, joint_names=joint_names) if q else None for q in solutions]

    def try_to_fit_configurations_between_bounds(self, robot, configurations, group=None):
        """
        """
        j1, j2, j3, j4, j5, j6 = robot.get_configurable_joints()
        for i, c in enumerate(configurations):
            if c is None:
                continue
            a1, a2, a3, a4, a5, a6 = c.values()
            try:
                a1 = fit_within_bounds(a1, j1.limit.lower, j1.limit.upper)
                a2 = fit_within_bounds(a2, j2.limit.lower, j2.limit.upper)
                a3 = fit_within_bounds(a3, j3.limit.lower, j3.limit.upper)
                a4 = fit_within_bounds(a4, j4.limit.lower, j4.limit.upper)
                a5 = fit_within_bounds(a5, j5.limit.lower, j5.limit.upper)
                a6 = fit_within_bounds(a6, j6.limit.lower, j6.limit.upper)
                configurations[i].joint_values = [a1, a2, a3, a4, a5, a6]
            except AssertionError:
                configurations[i] = None
        return configurations


if __name__ == "__main__":
    import compas_fab
    from compas_fab.robots import Tool
    from compas.datastructures import Mesh
    from compas.geometry import Point, Vector
    from compas_fab.robots import RobotSemantics

    from compas_fab.backends.kinematics.client import AnalyticalPyBulletClient

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))

    with AnalyticalPyBulletClient() as client:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

        # Load UR5
        robot = client.load_robot(urdf_filename)
        robot.semantics = RobotSemantics.from_srdf_file(srdf_filename, robot.model)

        # Update disabled collisions
        client.disabled_collisions = robot.semantics.disabled_collisions

        # Attach tool and convert frames
        robot.attach_tool(Tool(mesh, Frame((0.07, 0, 0), (0, 0, 1), (0, 1, 0)), name="light_pen"))
        frames_WCF = [Frame(Point(0.407, 0.073, 0.320), Vector(0.922, 0.000, 0.388), Vector(0.113, 0.956, -0.269)),
                      Frame(Point(0.404, 0.057, 0.324), Vector(0.919, 0.000, 0.394), Vector(0.090, 0.974, -0.210)),
                      Frame(Point(0.390, 0.064, 0.315), Vector(0.891, 0.000, 0.454), Vector(0.116, 0.967, -0.228)),
                      Frame(Point(0.388, 0.079, 0.309), Vector(0.881, 0.000, 0.473), Vector(0.149, 0.949, -0.278)),
                      Frame(Point(0.376, 0.087, 0.299), Vector(0.850, 0.000, 0.528), Vector(0.184, 0.937, -0.296))]
        frames_WCF_T0 = robot.attached_tool.from_tcf_to_t0cf(frames_WCF)

        # Now check each frame
        configurations_along_path = []

        for frame in frames_WCF_T0:
            configurations = list(robot.iter_inverse_kinematics(frame, options={"check_collision": True, "keep_order": True}))
            configurations_along_path.append(configurations)

        start_configurations = list(robot.iter_inverse_kinematics(frames_WCF_T0[0], options={"check_collision": True}))

        trajectory = robot.plan_cartesian_motion(frames_WCF_T0)
        j = [c.joint_values for c in trajectory.points]
        import matplotlib.pyplot as plt
        plt.plot(j)
        plt.show()
