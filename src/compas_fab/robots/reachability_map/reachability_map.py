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
    frames : list of list of :class:`compas.geometry.Frame`
        The frames at which the IK solutions are calculated.
    configurations : list of list of list of :class:`compas.robots.Configuration`
        The configurations at the frames.
    score : list of int
        The number of solutions per frame list (2D)
    points : list of :class:`compas.geometry.Point`
        The points per frame list (2D)
    shape : tuple of int
        The shape of the frames array


    Links
    -----
    http://wiki.ros.org/reuleaux
    """

    def __init__(self, frames=None, configurations=None, name=None):
        super(ReachabilityMap, self).__init__(name)
        self.frames = frames or []  # 2D
        self.configurations = configurations or []  # 3D

    def calculate(self, frame_generator, robot, ik_options=None):
        """Calculates the reachability map.

        Parameters
        ----------
        frame_generator : generator
            A 2D frame generator to yield :class:`compas.geometry.Frame`.
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
            This makes only sense if the robot has an analytic inverse kinematic
            solvers set.
        ik_options : dict, optional
            Optional arguments to be passed on to the robot's inverse kinematics
            function.

        Raises
        ------
        ValueError : If the frame_generator does not produce a 2D list of frames
        """

        from compas_fab.backends.exceptions import (
            InverseKinematicsError,
        )  # tests\api\test_api_completeness.py complains otherwise

        for frames in frame_generator:  # 2D
            if isinstance(frames, Frame):
                raise ValueError("Please pass a 2D frame generator")
            self.frames.append([])
            self.configurations.append([])
            for frame in frames:
                try:
                    configurations = [config for config in robot.iter_inverse_kinematics(frame, options=ik_options)]
                except InverseKinematicsError:
                    if "keep_order" in ik_options:
                        configurations = [None] * 8
                    else:
                        configurations = []
                self.frames[-1].append(frame)
                self.configurations[-1].append(configurations)

    @property
    def shape(self):
        dimension = []
        f = self.frames
        while not isinstance(f, Frame):
            dimension.append(len(f))
            f = f[0]
        return tuple(dimension)

    def reachable_frames_and_configurations_at_ik_index(self, ik_index):
        """Returns the reachable frames and configurations at a specific ik index."""
        configurations_at_ik_index = []
        frames_at_ik_index = []
        for f, c in zip(self.frames, self.configurations):
            for subf, subc in zip(f, c):
                configuration = subc[ik_index]
                if configuration:
                    configurations_at_ik_index.append(configuration)
                    frames_at_ik_index.append(subf)

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
        """Returns a 1D list of points from the first frame in the list."""
        return [f[0].point for f in self.frames]

    @property
    def data(self):
        def data_encode(obj):
            if isinstance(obj, list):
                return [data_encode(sub) for sub in obj]
            else:
                return obj.data if obj else None

        data = {}
        data["frames"] = data_encode(self.frames)
        data["configurations"] = data_encode(self.configurations)
        return data

    @data.setter
    def data(self, data):
        def data_decode(obj, aclass):
            if isinstance(obj, list):
                return [data_decode(sub, aclass) for sub in obj]
            else:
                return aclass.from_data(obj) if obj else None

        self.frames = data_decode(data["frames"], Frame)
        self.configurations = data_decode(data["configurations"], Configuration)
