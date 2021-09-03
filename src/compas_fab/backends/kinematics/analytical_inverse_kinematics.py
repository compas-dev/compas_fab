from compas.geometry import Frame
from compas.robots import Configuration
from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.kinematics.utils import fit_within_bounds
from compas_fab.backends.interfaces import InverseKinematics


class AnalyticalInverseKinematics(InverseKinematics):
    """Create a custom InverseKinematicsSolver for a robot.

    The ik for 6 axes industrial robots returns by default 8 possible solutions.
    Those solutions are also sorted. That means, the if you call ik
    on 2 frames that are close to each other, and compare the 8
    configurations of the first one with the 8 of the second one at
    their respective indices, then these configurations are 'close' to
    each other. That is why for certain use cases, e.g. custom cartesian
    path planning it makes sense to keep the sorting and set the ones
    that are out of joint limits or in collison to `None`.

    Examples
    --------

    >>> ik_solver = AnalyticalInverseKinematics()
    """

    def __init__(self, client=None):
        self.client = client

    def inverse_kinematics(self, robot, frame_RCF, start_configuration=None, group=None, options=None):

        solutions = self._inverse_kinematics(frame_RCF)

        configurations = self.joint_angles_to_configuration(robot, solutions)

        # check collisions for all configurations (sets those to `None` that are not working)
        if options and "check_collision" in options and options["check_collision"] is True:
            for i, config in enumerate(configurations):
                try:
                    self.client.check_collisions(robot, config)
                except BackendError:
                    configurations[i] = None

        # fit configurations within joint bounds (sets those to `None` that are not working)
        configurations = self.try_to_fit_configurations_between_bounds(robot, configurations)

        # removes the `None` ones
        if options and "cull" in options and options["cull"] is True:
            configurations = [c for c in configurations if c is not None]

        return configurations

    def _inverse_kinematics(self, frame):
        raise NotImplementedError

    def joint_angles_to_configurations(self, robot, solutions):
        joint_names = robot.get_configurable_joint_names()
        return [Configuration.from_revolute_values(q, joint_names=joint_names) for q in solutions]

    def try_to_fit_configurations_between_bounds(self, robot, configurations):
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


class UR5_Analytical_IK(AnalyticalInverseKinematics):

    def _inverse_kinematics(self, frame):
        from compas_fab.backends.kinematics.offset_wrist_kinematics import UR5
        return UR5().inverse(frame)


if __name__ == "__main__":
    import time
    import compas_fab
    from compas_fab.robots import Tool
    from compas.datastructures import Mesh
    from compas.geometry import Point, Vector
    from compas_fab.robots import RobotSemantics
    from compas_fab.backends import PyBulletClient

    class Client(PyBulletClient):
        def inverse_kinematics(self, *args, **kwargs):
            return UR5_Analytical_IK(self)(*args, **kwargs)

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))

    with Client() as client:
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
            # the following is not working because in robot we split joint_names and joint_values and do not support lists as outcomes
            # configurations = robot.inverse_kinematics(frame_WCF)
            configurations = client.inverse_kinematics(robot, frame, options={"check_collision": True})
            configurations_along_path.append(configurations)

        # with a bit of work we can turn this into a cartesian planner that returns up to 8 possible paths
        paths = []
        for configurations in zip(*configurations_along_path):
            if all(configurations):
                paths.append(configurations)

        # visualize
        for path in paths:
            for configuration in path:
                frame_WCF = robot.forward_kinematics(configuration, options={"check_collision": True})
                print(frame_WCF)
                time.sleep(0.1)
            print("=")
