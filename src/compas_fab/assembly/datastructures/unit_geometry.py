"""An abstract geometry object that can be made as parametric geometry class
or a determinate mesh object

"""
from compas_fab.backends.pybullet import pb_pose_from_Frame

class UnitGeometry(object):
    def __init__(self, name, mesh=None, body=None,
                 initial_frame=None, goal_frame=None,
                 grasps=[], initial_supports=[], goal_supports=[]):
        self._name = name
        self._mesh = mesh
        self._body = body
        self._initial_frame = initial_frame
        self._goal_frame = goal_frame
        self._grasps = grasps
        self._initial_supports = initial_supports
        self._goal_supports = goal_supports

    @property
    def name(self):
        return self._name

    @property
    def pybullet_bodies(self):
        # TODO: if no body is stored, create from compas mesh
        return self._body

    @pybullet_bodies.setter
    def pybullet_bodies(self, body):
        self._body = body

    @property
    def mesh(self):
        # TODO: if no mesh is stored, but pybullet body is assigned
        # create from pybullet body
        return self._mesh

    @mesh.setter
    def mesh(self, input_mesh):
        self._mesh = input_mesh

    @property
    def grasps(self):
        return self._grasps

    @grasps.setter
    def grasps(self, input_grasps):
        self._graps = input_grasps

    @property
    def initial_frame(self):
        return self._initial_frame

    @initial_frame.setter
    def initial_frame(self, frame):
        self._initial_frame = frame

    @property
    def initial_pb_pose(self):
        return pb_pose_from_Frame(self._initial_frame)

    @property
    def goal_frame(self):
        # TODO: if no body is stored, create from compas mesh
        return self._goal_frame

    @goal_frame.setter
    def goal_frame(self, frame):
        self._goal_frame = frame

    @property
    def goal_pb_pose(self):
        return pb_pose_from_Frame(self._goal_frame)

    # --------------------------------------------------------------------------
    # attributes
    # --------------------------------------------------------------------------

    def centroid(self):
        raise NotImplementedError
