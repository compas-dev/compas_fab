from compas.data import Data
from compas.geometry import Frame
from compas.geometry import argmax
from compas_robots import Configuration
from ..targets import FrameTarget


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
    configurations : list of list of list of :class:`compas_robots.Configuration`
        The configurations at the frames.
    score : list of int
        The number of solutions per frame list (2D)
    points : list of :class:`compas.geometry.Point`
        The points per frame list (2D)
    shape : tuple of int
        The shape of the frames array

    Notes
    -----
    See Also `reuleaux <https://wiki.ros.org/reuleaux>`__
    """

    def __init__(self, frames=None, configurations=None, name=None):
        super(ReachabilityMap, self).__init__(name)
        self.frames = frames or []  # 2D
        self.configurations = configurations or []  # 3D

    def calculate(self, frame_generator, planner, robot_cell_state, ik_options=None):
        """Calculates the reachability map for a robot cell.

        The robot_cell must be set in the planner before calling this function by calling
        `planner.set_robot_cell(robot_cell)`

        Collision checking is only available if the planner supports it. For example,
        AnalyticalPyBulletPlanner supports collision checking while AnalyticalKinematicsPlanner does not.

        If tools are attached to the robot, the robot cell must contain the tool and robot cell state must reflect this.
        The frame generator will generate frames for the Tool Coordinate Frame (TCF).
        If no tools are attached, the robot's Planner Coordinate Frame (PCF) is used.

        Parameters
        ----------
        frame_generator : generator
            A 2D frame generator to yield :class:`compas.geometry.Frame`.
        planner : :class:`compas_fab.backends.PlanningInterface`
            The planner backend for which inverse kinematics is being calculated.
            It only makes only sense to use AnalyticalPyBulletPlanner or AnalyticalKinematicsPlanner
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state for which the reachability map is calculated.
            The initial robot configuration is not important.
        ik_options : dict, optional
            Optional arguments to be passed on to the planner's inverse kinematics
            function.

        Raises
        ------
        ValueError : If the frame_generator does not produce a 2D list of frames
        """

        from compas_fab.backends.exceptions import (
            InverseKinematicsError,
        )  # tests\api\test_api_completeness.py complains otherwise

        # Set Robot Cell State
        planner.set_robot_cell_state(robot_cell_state)

        for frames in frame_generator:  # 2D
            if isinstance(frames, Frame):
                raise ValueError("Please pass a 2D frame generator")
            self.frames.append([])
            self.configurations.append([])
            for frame in frames:
                try:
                    # Tolerance settings in the target is left as default
                    target = FrameTarget(frame)
                    # TODO: Test to make sure the IK uses TCF when tools are attached
                    configurations = [
                        config
                        for config in planner.iter_inverse_kinematics(target, robot_cell_state, options=ik_options)
                    ]
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
    def __data__(self):
        def _recursive_encode(obj):
            if isinstance(obj, list):
                return [_recursive_encode(sub) for sub in obj]
            else:
                return obj.__data__ if obj else None

        data = {}
        data["frames"] = _recursive_encode(self.frames)
        data["configurations"] = _recursive_encode(self.configurations)
        data["name"] = self.name
        return data

    @classmethod
    def __from_data__(cls, data):
        def _recursive_decode(obj, aclass):
            if isinstance(obj, list):
                return [_recursive_decode(sub, aclass) for sub in obj]
            else:
                return aclass.__from_data__(obj) if obj else None

        frames = _recursive_decode(data["frames"], Frame)
        configurations = _recursive_decode(data["configurations"], Configuration)
        name = data.get("name", None)
        return cls(frames, configurations, name)
