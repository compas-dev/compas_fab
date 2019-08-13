from compas_fab.backends.pybullet import pb_pose_from_Frame

class Grasp(object):
    def __init__(self, object_index, grasp_id, approach, attach, retreat):
        self._object_index = object_index # brick index
        self._grasp_id = grasp_id # grasp id
        self._object_from_approach_frame = approach # compas Frame
        self._object_from_attach_frame = attach # compas Frame
        self._object_from_retreat_frame = retreat # compas Frame

    @property
    def object_from_approach_pb_pose(self):
        return pb_pose_from_Frame(self._object_from_approach_frame)

    @property
    def object_from_attach_pb_pose(self):
        return pb_pose_from_Frame(self._object_from_attach_frame)

    @property
    def object_from_retreat_pb_pose(self):
        return pb_pose_from_Frame(self._object_from_retreat_frame)

    def __repr__(self):
        return '{}(body index #{}, grasp id #{})'.format(self.__class__.__name__, self._object_index, self._grasp_id)
