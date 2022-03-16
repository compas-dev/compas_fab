from compas.data import Data
from compas.geometry import Frame
from compas.robots import Configuration
from compas.geometry import argmax


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
        depending on the dimension of the frames respectively.
    score : list of int
        The number of solutions per frame (1D) or per frame list (2D)
    points : list of :class:`compas.geometry.Point`
        The points per frame (1D) or per frame list (2D)
    shape : tuple of int
        The shape of the frame array


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
            The solutions are saved depending on the dimensions.
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
            This makes only sense if the robot has an analytic inverse kinematic
            solvers set.
        ik_options : dict, optional
            Optional arguments to be passed on to the robot's inverse kinematics
            function.
        """

        from compas_fab.backends.exceptions import InverseKinematicsError

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
    def shape(self):
        dimension = []
        f = self.frames
        while not isinstance(f, Frame):
            dimension.append(len(f))
            f = f[0]
        return tuple(dimension)

    def reachable_frames_and_configurations_at_ik_index(self, ik_index):
        """Returns the reachable frames and configurations at a specific ik index.
        """

        def check_and_append(f, c, flist, clist):
            if isinstance(f, Frame):
                configuration = c[ik_index]
                if configuration:
                    clist.append(configuration)
                    flist.append(f)
            else:
                for subf, subc in zip(f, c):
                    check_and_append(subf, subc, flist, clist)

        configurations_at_ik_index = []
        frames_at_ik_index = []
        for f, c in zip(self.frames, self.configurations):
            check_and_append(f, c, frames_at_ik_index, configurations_at_ik_index)
        return frames_at_ik_index, configurations_at_ik_index

    @property
    def score(self):
        def sum_score(obj):
            if isinstance(obj, list):
                return sum([sum_score(sub) for sub in obj])
            else:
                return 1 if obj else 0
        return [sum_score(configuration) for configuration in self.configurations]

    @property
    def best_score(self):
        score = self.score
        return max(score), argmax(score)

    @property
    def points(self):
        """Returns a 1D list of points.

        If self.frames is a 2D list, the point of the first frame is returned.
        """
        def get_first_point(f):
            while not isinstance(f, Frame):
                f = f[0]
            return f.point
        return [get_first_point(f) for f in self.frames]

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
