from compas.data import Data
from compas.geometry import Frame
from compas.robots import Configuration
from compas_fab.backends.exceptions import InverseKinematicsError


class ReachabilityMap(Data):
    """The ReachabilityMap describes the reachability of a robot.

    The ReachabilityMap describes the reachability of a robot at certain frames,
    with valid IK solutions at these frames. The map only makes sense to be
    calculated with analytic inverse kinematic solvers, as they include all
    possible solutions to be found.

    Attributes
    ----------
    frames : list of :class:`compas.geometry.Frame` or list of list of :class:`compas.geometry.Frame`
        The frames at which the IK solutions are calculated, either a 1D list or
        a 2D list.
    configurations : list of list of :class:`compas.robots.Configuration` or list of list of list of :class:`compas.robots.Configuration`
        The configurations at the frames, either a 2D list or a 3D list,
        depending on the frames respectively.
    score : list of int
        The number of solutions per frame.
    points : list of :class:`compas.geometry.Point`
        The points per frame.

    Notes
    -----
    This works only for industrial robot arms with six revolute joints.
    If ``check_collision`` is `True`, it is required to use a client
    that supports ``"check_collision"``, so for now only the `PyBulletClient`.


    Links
    -----
    http://wiki.ros.org/reuleaux
    """

    def __init__(self, frames=None, configurations=None, name=None):
        super(ReachabilityMap, self).__init__(name)
        self.frames = frames or []  # 1D or 2D
        self.configurations = configurations or []  # 2D or 3D

    def calculate(self, frame_generator, robot, ik_options=None):
        """Calculates the reachability map.

        Parameters
        ----------
        frame_generator : generator
            A 1D or 2D frame generator to yield :class:`compas.geometry.Frame`.
            The solutions are saved depending on the dimensionality.
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
            This makes only sense if the robot has an analytic inverse kinematic
            solvers set.
        ik_options : dict
            Optional arguments to be passed on to the robot's inverse kinematics
            function.
        """

        def calculate_and_append(obj, flist, clist):
            if isinstance(obj, Frame):
                try:
                    configurations = [config for config in robot.iter_inverse_kinematics(obj, options=ik_options)]
                except InverseKinematicsError:
                    if "keep_order" in ik_options:
                        configurations = [None for _ in range(8)]
                    else:
                        configurations = []
                flist.append(obj)
                clist.append(configurations)
            else:
                flist.append([])
                clist.append([])
                for sub in obj:
                    calculate_and_append(sub, flist[-1], clist[-1])

        for frame in frame_generator:  # 1D or 2D
            calculate_and_append(frame, self.frames, self.configurations)

    @property
    def score(self):
        def sum_score(obj):
            if isinstance(obj, list):
                return sum([sum_score(sub) for sub in obj])
            else:
                return 1 if obj else 0
        return [sum_score(configuration) for configuration in self.configurations]

    @property
    def points(self):
        if isinstance(self.frames[0], Frame):
            return [frame.point for frame in self.frames]
        else:
            return [frames_per_point[0].point for frames_per_point in self.frames]

    @property
    def data(self):
        def data_encode(obj):
            if isinstance(obj, list):
                return [data_encode(sub) for sub in obj]
            else:
                return obj.data if obj else None

        data = {}
        data['frames'] = data_encode(self.frames)
        data['configurations'] = data_encode(self.configurations)
        return data

    @data.setter
    def data(self, data):
        def data_decode(obj, aclass):
            if isinstance(obj, list):
                return [data_decode(sub, aclass) for sub in obj]
            else:
                return aclass.from_data(obj) if obj else None

        self.frames = data_decode(data['frames'], Frame)
        self.configurations = data_decode(data['configurations'], Configuration)


if __name__ == "__main__":

    import os
    import math
    from compas.geometry import Frame, Vector
    from compas_fab.backends import AnalyticalInverseKinematics
    from compas_fab.backends import PyBulletClient
    from compas_fab import DATA

    from compas_fab.robots.reachability_map.frame_generator import DeviationVectorsGenerator
    from compas_fab.robots.reachability_map.frame_generator import OrthonormalVectorsFromAxisGenerator

    frames = []
    frame = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    step = 0.05
    for i in range(10):
        point = frame.point + Vector(1, 0, 0) * (step * i)
        frames.append(Frame(point, frame.xaxis, frame.yaxis))

    # Version 1: iterate over frames along a line, frames = 1D list

    with PyBulletClient(connection_type='direct') as client:
        robot = client.load_ur5(load_geometry=True)

        ik = AnalyticalInverseKinematics(client)
        # set a new IK function
        client.inverse_kinematics = ik.inverse_kinematics

        options = {"solver": "ur5", "check_collision": True, "keep_order": True}

        map = ReachabilityMap()
        map.calculate(frames, robot, options)
        map.to_json(os.path.join(DATA, "reachability", "map1D.json"))
        #map = ReachabilityMap.from_json(os.path.join(DATA, "reachability", "map1D.json"))
        # print("=====================")
        # print(map.frames)
        # print(map.configurations)

    print(map.score)

    """

    # Version 2: iterate over frames around each point on the line, frames = 2D list

    def generator(frames):

        def gen(pt):
            for zaxis in DeviationVectorsGenerator((0, 0, -1), math.radians(90), 1):
                for xaxis in OrthonormalVectorsFromAxisGenerator(zaxis, math.radians(90)):
                    yaxis = zaxis.cross(xaxis)
                    yield Frame(pt, xaxis, yaxis)

        for f in frames:
            yield gen(f.point)

    with PyBulletClient(connection_type='direct') as client:
        robot = client.load_ur5(load_geometry=True)

        ik = AnalyticalInverseKinematics(client)
        # set a new IK function
        client.inverse_kinematics = ik.inverse_kinematics

        options = {"solver": "ur5", "check_collision": True, "keep_order": True}

        map = ReachabilityMap()
        map.calculate(generator(frames), robot, options)
        map.to_json(os.path.join(DATA, "reachability", "map2D.json"))
        #map = ReachabilityMap.from_json(os.path.join(DATA, "reachability", "map2D.json"))
        #print("=====================")
        #print(map.frames)
        #print(map.configurations)
    """
