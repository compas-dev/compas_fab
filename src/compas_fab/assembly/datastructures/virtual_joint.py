from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# import json

from compas.geometry import Frame
from compas.datastructures import Mesh

__all__ = ['VirtualJoint']


class VirtualJoint(object):

    __module__ = 'compas_fab.assembly.datastructures'

    def __init__(self):
        super(VirtualJoint, self).__init__()
        self._connected_element_ids = None

    @property
    def connected_element_ids(self):
        return self._connected_element_ids

    @connected_element_ids.setter
    def connected_element_ids(self, e_ids):
        self._connected_element_ids = e_ids

    # --------------
    # constructors
    # --------------
#     @classmethod
#     def create_from_object_place_pose(cls, id, rbase_frame, \
#             rgeometry_in_scene, rpose_in_scene):
#         """create object from geometry in a scene pose
#
#         We perform transformation here to orient the geometry back to
#         the base_link
#
#         Parameters
#         ----------
#         rbase_frame : rhion plane
#         rgeometry_in_scene: list of rhino meshes
#         rpose_in_scene: rhino planes
#         """
#         as_obj = cls()
#         as_obj.base_frame = rPln2cFrame(rbase_frame)
#         as_obj.shape_place_pose = rPln2cFrame(rpose_in_scene)
#
#         # perform transformation on the geometry_in_scene
#         obj_tf = Transformation.from_frame(as_obj.shape_place_pose)
#
#         cmesh_tf = [transform_cmesh(rMesh2cMesh(rm), obj_tf.inverse()) \
#                 for rm in rgeometry_in_scene]
#
#         as_obj.shape = cmesh_tf
#
#         return as_obj
#
#     # --------------
#     # attributes assigners
#     # --------------
#     def set_pick_in_scene_pose(self, rpick_plane):
#         self.shape_pick_pose = rPln2cFrame(rpick_plane)
#
#     def set_place_in_scene_pose(self, rplace_plane):
#         self.shape_place_pose = rPln2cFrame(rplace_plane)
#
#     def set_ee_poses_from_in_scene_poses(self, rworld_from_obj_pose, rworld_from_ee_poses):
#         """create obj_from_grasp poses from world
#
#         Parameters
#         ----------
#         rworld_from_obj_pose: rhino plane
#         rworld_from_ee_poses: a list of rhino plane
#         """
#         world_obj_tf = Transformation.from_frame( \
#             rPln2cFrame(rworld_from_obj_pose))
#
#         self.object_from_ee_poses = []
#         for world_ee_fr in [rPln2cFrame(rw_ee) for rw_ee in rworld_from_ee_poses]:
#             w_ee_tf = Transformation.from_frame(world_ee_fr)
#             self.object_from_ee_poses.append(Frame.from_transformation( \
#                 Transformation.concatenate(world_obj_tf.inverse(), w_ee_tf)))
#
#
#     def set_ee_place_poses_from_obj(self, obj_from_ee_poses):
#         assert(False, "not implemented")
#
#     @property
#     def world_from_pick_ee_poses(self):
#         """return a list of (world_from) ee pick poses
#         """
#         assert(self.shape_pick_pose != None, "pick pose not defined!")
#         world_from_ee_pick = []
#         world_pick_tf = Transformation.from_frame(self.shape_pick_pose)
#         for obj_from_ee in self.object_from_ee_poses:
#             w_f_ee_tf = Transformation.concatenate(world_pick_tf, Transformation.from_frame(obj_from_ee))
#             world_from_ee_pick.append(Frame.from_transformation(w_f_ee_tf))
#         return world_from_ee_pick
#
#     @property
#     def world_from_place_ee_poses(self):
#         """return a list of (world_from) ee place poses
#         """
#         assert(self.shape_place_pose != None, "place pose not defined!")
#         world_from_ee_place = []
#         world_place_tf = Transformation.from_frame(self.shape_place_pose)
#         for obj_from_ee in self.object_from_ee_poses:
#             w_f_ee_tf = Transformation.concatenate(world_place_tf, Transformation.from_frame(obj_from_ee))
#             world_from_ee_place.append(Frame.from_transformation(w_f_ee_tf))
#         return world_from_ee_place
#
#     @property
#     def shape_in_pick_pose(self):
#         """return shape geometries in pick pose"""
#         assert(self.shape_pick_pose != None, "pick pose not defined!")
#         world_pick_tf = Transformation.from_frame(self.shape_pick_pose)
#         return [transform_cmesh(cm, world_pick_tf) for cm in self.shape]
# #        return [cm for cm in self.shape]
#
#     @property
#     def shape_in_place_pose(self):
#         """return shape geometries in pick pose"""
#         assert(self.shape_place_pose != None, "place pose not defined!")
#         world_place_tf = Transformation.from_frame(self.shape_place_pose)
#         return [transform_cmesh(cm, world_place_tf) for cm in self.shape]

# ==============================================================================
# Main
# ==============================================================================

if __name__ == "__main__":
    pass
