from compas.geometry import Frame
from compas.robots import Configuration
from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.kinematics.utils import fit_within_bounds
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet import PyBulletClient





class InverseKinematicsAnalytical(InverseKinematics):
    """Create a custom InverseKinematicsSolver for a robot.

    Examples
    --------

    >>> ik_solver = InverseKinematicsSolver(robot, "robotA", ik_abb_irb4600_40_255)
    >>> robot.inverse_kinematics = ik_solver.inverse_kinematics_function()
    >>> ikfast_fn = get_ik_fn_from_ikfast(ikfast_abb_irb4600_40_255.get_ik)
    >>> ik_solver = InverseKinematicsSolver(robot, "robotA", ikfast_fn)
    """

    def __init__(self, robot, start_configuration=None, group=None):
        self.robot = robot
        #self.group = group or robot.main_group_name
        #self.start_configuration = start_configuration
        #self.joint_names = self.robot.get_configurable_joint_names(self.group)

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):

        start_configuration = start_configuration or self.start_configuration  # todo update if passed
        group = group or self.group  # todo update if passed

        frame = frame_WCF
        if self.robot.attached_tool:
            frame = self.robot.attached_tool.from_tcf_to_t0cf([frame])[0]  # todo precalc transforms?

        if start_configuration:
            base_frame = self.robot.get_base_frame(group, full_configuration=start_configuration)
            frame = base_frame.to_local_coordinates(frame)

        # frame_tool0_RCF = Frame.from_transformation(self.base_transformation * Transformation.from_frame(frame_WCF) * self.tool_transformation)

        A1, A2, A3, A4, A5, A6 = self._inverse_kinematics(frame)

        # The ik solution for 6 axes industrial robots returns by default 8
        # configurations, which are sorted. That means, the if you call ik
        # on 2 frames that are close to each other, and compare the 8
        # configurations of the first one with the 8 of the second one at
        # their respective indices, then these configurations are 'close' to
        # each other. That is why for certain use cases, e.g. custom cartesian
        # path planning it makes sense to keep the sorting and set the ones
        # that are out of joint limits or in collison to `None`.

        configurations = self.joint_angles_to_configuration(A1, A2, A3, A4, A5, A6)

        # check collisions for all configurations (sets those to `None` that are not working)
        if options and "check_collision" in options and options["check_collision"] == True:
            for i, config in enumerate(configurations):
                try:
                    self.client.check_collisions(robot, config)
                except BackendError:
                    configurations[i] = None
        
        # fit configurations within joint bounds (sets those to `None` that are not working)
        self.try_to_fit_configurations_between_bounds(configurations)
        
        

        """

        if return_idxs:
            configurations = [configurations[i] for i in return_idxs]

        # add joint names to configurations
        self.add_joint_names_to_configurations(configurations)

        if return_closest_to_start:
            diffs = [c.max_difference(start_configuration) for c in configurations if c is not None]
            if len(diffs):
                idx = diffs.index(min(diffs))
                return configurations[idx]  # only one
            return None

        if cull:
            configurations = [c for c in configurations if c is not None]

        """
        return config.values, config.joint_names

    def _inverse_kinematics(self, frame):
        pass

    def joint_angles_to_configuration(self, A1, A2, A3, A4, A5, A6):
        return [Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6], joint_names=self.joint_names) for a1, a2, a3, a4, a5, a6 in zip(A1, A2, A3, A4, A5, A6)]

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


class UR5Kinematics(InverseKinematicsAnalytical):

    def _inverse_kinematics(self, frame):
        """
        from .offset_wrist_kinematics import UR3_Kinematics
        from .offset_wrist_kinematics import UR5_Kinematics
        from .offset_wrist_kinematics import UR10_Kinematics
        from .spherical_wrist_kinematics import Staubli_TX2_60L_Kinematics
        from .spherical_wrist_kinematics import ABB_IRB_4600_40_255_Kinematics
        """
        from compas_fab.backends.kinematics.offset_wrist_kinematics import UR5
        return UR5().inverse(frame)


if __name__ == "__main__":
    import compas_fab
    from compas_fab.robots.ur5 import Robot
    from compas_fab.robots import Tool
    from compas.datastructures import Mesh
    from compas.geometry import Point, Vector
    from compas.robots import LocalPackageMeshLoader
    from compas_fab.robots import RobotSemantics

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    robot = Robot(load_geometry=True)
    robot.attach_tool(Tool(mesh, Frame((0.07, 0, 0), (0, 0, 1), (0, 1, 0)), name="light_pen"))

    start_configuration = robot.zero_configuration()

    frames_WCF = [Frame(Point(0.407, 0.073, 0.320), Vector(0.922, 0.000, 0.388), Vector(0.113, 0.956, -0.269)), Frame(Point(0.404, 0.057, 0.324), Vector(0.919, 0.000, 0.394), Vector(0.090, 0.974, -0.210)), Frame(Point(0.390, 0.064, 0.315), Vector(0.891, 0.000, 0.454), Vector(0.116, 0.967, -0.228)), Frame(Point(0.388, 0.079, 0.309), Vector(0.881, 0.000, 0.473), Vector(0.149, 0.949, -0.278)), Frame(Point(0.376, 0.087, 0.299), Vector(0.850, 0.000, 0.528), Vector(0.184, 0.937, -0.296))]
    start_configuration = robot.zero_configuration()

    frames_WCF_T0 = robot.attached_tool.from_tcf_to_t0cf(frames_WCF)

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')

    disabled_collisions = robot.semantics.disabled_collisions


    loader = LocalPackageMeshLoader(compas_fab.get('universal_robot'), 'ur_description')

    from compas_fab.backends import PyBulletClient


    class Client(PyBulletClient):
        def inverse_kinematics(self, *args, **kwargs):
            return UR5Kinematics(self)(*args, **kwargs)

    # So that usage would be:
    #with Client() as client:
    #robot = client.load_robot(path_to_urdf_file)
    #some_frame = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])

    #print(robot.inverse_kinematics(some_frame))


    #with PyBulletClient() as client:
    #urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    #robot = client.load_robot(urdf_filename)

    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

    frame_WCF = robot.forward_kinematics(configuration)


    print("Frame in the world coordinate system")
    print(frame_WCF)
    import time

    with Client() as client: #connection_type='direct'

        robot = client.load_robot(urdf_filename)
        client.disabled_collisions = disabled_collisions
        #print(client.inverse_kinematics(robot, frames_WCF_T0[0], start_configuration))

        print(type(robot))
        #robot.client = client
        
        """
        cached_robot_file_name = str(robot.model.guid) + '.urdf'

        client.cache_robot(robot)
        urdf_fp = robot.attributes['pybullet']['cached_robot_filepath']

        client._load_robot_to_pybullet(urdf_fp, robot)
        #client.load_robot(cached_robot_file_name)

        #print(client.plan_cartesian_motion(frames_WCF, start_configuration))

        print(robot.inverse_kinematics(frames_WCF_T0[0], start_configuration))
        """

        #print(client.check_collisions(robot, start_configuration))
        #print(robot.forward_kinematics(start_configuration, options={"check_collision" : True}))
        #print(start_configuration)
        print(robot.forward_kinematics(start_configuration, options={"check_collision" : True}))
        #time.sleep(25)
        #print(client.check_collisions(robot, start_configuration))