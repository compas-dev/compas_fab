from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# import json
# import pickle

from compas.geometry import Frame, Transformation
from .utils import element_vert_key, extract_element_vert_id

__all__ = ['Element']

class Element(object):

    __module__ = 'compas_fab.assembly.datastructures'

    def __init__(self, id):
        super(Element, self).__init__()
        self._parent_frame = Frame.worldXY() # TODO
        self._key = element_vert_key(id)
        self._world_from_element_place_pose = None
        self._world_from_element_pick_pose = None
        self._obj_from_grasp_poses = None

    @property
    def key(self):
        return self._key

    @property
    def key_id(self):
        return extract_element_vert_id(self.key)

    @property
    def parent_frame(self):
        return self._parent_frame

    @property
    def world_from_element_place_pose(self):
        return self._world_from_element_place_pose

    @world_from_element_place_pose.setter
    def world_from_element_place_pose(self, pose):
        self._world_from_element_place_pose = pose

    @property
    def world_from_element_pick_pose(self):
        return self._world_from_element_pick_pose

    @world_from_element_pick_pose.setter
    def world_from_element_pick_pose(self, pose):
        self._world_from_element_pick_pose = pose

    @property
    def obj_from_grasp_poses(self):
        return self._obj_from_grasp_poses

    @obj_from_grasp_poses.setter
    def obj_from_grasp_poses(self, grasp_poses):
        self._obj_from_grasp_poses = grasp_poses

    def set_grasp_poses_from_in_scene_poses(self, world_from_obj_pose, world_from_ee_poses):
        """create obj_from_grasp poses from world
        """
        world_obj_tf = Transformation.from_frame(world_from_obj_pose)
        self.obj_from_grasp_poses = []
        for world_ee_fr in world_from_ee_poses:
            w_ee_tf = Transformation.from_frame(world_ee_fr)
            self.obj_from_grasp_poses.append(Frame.from_transformation(\
                Transformation.concatenate(world_obj_tf.inverse(), w_ee_tf)))


    @property
    def world_from_pick_ee_poses(self):
        """return a list of (world_from) ee pick poses
        """
        assert(self.world_from_element_pick_pose != None, "pick pose not defined!")
        world_from_ee_pick = []
        world_pick_tf = Transformation.from_frame(self.world_from_element_pick_pose)
        for obj_from_ee in self.object_from_ee_poses:
            w_f_ee_tf = Transformation.concatenate(world_pick_tf, Transformation.from_frame(obj_from_ee))
            world_from_ee_pick.append(Frame.from_transformation(w_f_ee_tf))
        return world_from_ee_pick

    @property
    def world_from_place_ee_poses(self):
        """return a list of (world_from) ee place poses
        """
        assert(self.world_from_element_place_pose != None, "place pose not defined!")
        world_from_ee_place = []
        world_place_tf = Transformation.from_frame(self.world_from_element_place_pose)
        for obj_from_ee in self.object_from_ee_poses:
            w_f_ee_tf = Transformation.concatenate(world_place_tf, Transformation.from_frame(obj_from_ee))
            world_from_ee_place.append(Frame.from_transformation(w_f_ee_tf))
        return world_from_ee_place


# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":
    pass
