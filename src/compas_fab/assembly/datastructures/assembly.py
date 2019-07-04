from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import os
import json
from collections import OrderedDict

from compas.datastructures.network import Network
from compas.geometry import Frame, cross_vectors, Transformation

from .element import Element
from .virtual_joint import VirtualJoint
from .utils import obj_name, cframe2json, transform_cmesh, element_vert_key, \
    virtual_joint_key


__all__ = ['Assembly']

# the default description frame
WORLD_FRAME = Frame.worldXY()

class Assembly(object):
    """A data structure for discrete element assemblies.

    An assembly is composed of:
        - a collection of discrete element geometries
        - a collection of physical joint geometries (optional)
        - a network modeling connectivity and interface info between elements and joints
    There are two types of vertices in this network:
        - element_vert
            represents unit assembly element
        - virtual_joint_vert
            represents a connection between elements. But it does not necessarily
            refer to a phyiscal joint. For example in the case of compression-only
            blocks, the virtual joint models contact information between two blocks,
            where no real joint exists between these two blocks.

    """

    __module__ = 'compas_assembly.datastructures'

    def __init__(self, elements=None, attributes=None):
        super(Assembly, self).__init__()
        self._net = Network()
        self._element_geometries = dict() # object frame moved to world frame
        self._virtual_joint_geometries = dict()
        self._num_of_elements = 0
        self._num_of_virtual_joints = 0

    @property
    def network(self):
        return self._net

    def add_element(self, element_instance, id, unit_geometry):
        # unit geometry's object frame transformed to origin
        self._net.add_vertex(key=element_vert_key(id), element=element_instance, tag='element')
        self._element_geometries[element_vert_key(id)] = unit_geometry
        self._num_of_elements += 1

    def add_virtual_joint(self, vj_instance, id, unit_geometry=None):
        self._net.add_vertex(key=virtual_joint_key(id), virtual_joint=vj_instance, tag='virtual_joint')
        connected_e_ids = vj_instance.connected_element_ids
        for e_id in connected_e_ids:
            assert(element_vert_key(e_id) in self._net.vertex)
            self._net.add_edge(element_vert_key(e_id), virtual_joint_key(id))
        self._virtual_joint_geometries[virtual_joint_key(id)] = unit_geometry
        self._num_of_virtual_joints += 1

    def get_connected_virtual_joint(self, element_ids):
        # return virtual joint by connected element_ids
        pass

    def get_virtual_joint(self, vj_id):
        # network.vertices[_num_of_elements-1 + vj_id]
        pass

    def get_element(self, e_id):
        # assert(element_vert_key(e_id) in self._net.vertex)
        return self._net.get_vertex_attribute(element_vert_key(e_id), 'element')

    def get_element_neighbored_elements(self, e_id):
        # network neighbor, check vertex tag
        pass

    def get_element_neighbored_virtual_joints(self, e_id):
        # network neighbor, check vertex tag
        pass

    def element_geometry_in_pick_pose(self, e_id):
        """return shape geometries in pick pose"""
        e = self.get_element(e_id)
        assert(e.world_from_element_pick_pose != None, "pick pose not defined!")
        world_pick_tf = Transformation.from_frame(e.world_from_element_pick_pose)
        return [transform_cmesh(cm, world_pick_tf) \
        for cm in self._element_geometries[element_vert_key(e_id)]]

    def element_geometry_in_place_pose(self, e_id):
        """return shape geometries in place pose"""
        e = self.get_element(e_id)
        assert(e.world_from_element_place_pose != None, "place pose not defined!")
        world_place_tf = Transformation.from_frame(e.world_from_element_place_pose)
        return [transform_cmesh(cm, world_place_tf) \
        for cm in self._element_geometries[element_vert_key(e_id)]]

    # @classmethod
    # def from_network_and_assembly_objects(cls, net, objs):
    #     assert(net.number_of_vertices()==len(objs), \
    #         "assembly objects and network vertices not aligned!")
    #     as_instance = cls.from_network(net)
    #
    #     # assign assembly object attribute
    #     for i, vert_key in enumerate(as_instance.as_net.vertices()):
    #         objs[i].id = i
    #         as_instance.as_net.set_vertex_attribute(vert_key, "assembly_object", objs[i])
    #     return as_net

#     # --------------
#     # exporters
#     # --------------
#     def save_assembly_object_to_objs(self, mesh_path):
#         if not os.path.isdir(mesh_path):
#             os.mkdir(mesh_path)
#
#         for as_obj in self.assembly_objects:
#             for i, s in enumerate(as_obj.shape):
#                 s.to_obj(os.path.join(mesh_path, obj_name(as_obj.id, i)))
#
#     def save_assembly_network_to_json(self, json_path, pkg_name='', \
#             assembly_type='', model_type='', unit=''):
#         if not os.path.isdir(json_path):
#             os.mkdir(json_path)
#
#         json_file_path = os.path.join(json_path, pkg_name + '.json')
#
#         data = OrderedDict()
#         data['pkg_name'] = pkg_name
#         data['assembly_type'] = assembly_type
#         data['model_type'] = model_type
#         data['unit'] = unit
#
#         data['element_number'] = self.number_of_vertices()
#         data['sequenced_elements'] = []
#
#         for v in self.vertices():
#             assert(v == self.assembly_object(v).id)
#             ao = self.assembly_object(v)
#
#             e_data = OrderedDict()
#             e_data['order_id'] = v
#             e_data['base_frame'] = cframe2json(ao.base_frame)
#             e_data['element_geometry_file_names'] = [obj_name(v, sub_id) \
#                 for sub_id in range(len(ao.shape))]
#
#             grasp_data = OrderedDict()
#             grasp_data['parent_link'] = 'object' #world
#             grasp_data['ee_poses'] = [cframe2json(ee_p) \
#                 for ee_p in ao.object_from_ee_poses]
#
#             pick = OrderedDict()
#             pick['process_name'] = 'pick'
#             pick['base_frame'] = cframe2json(ao.base_frame)
#             pick['target_pose'] = cframe2json(ao.shape_pick_pose)
#             pick['allowed_collision_obj_names'] = [] # support tables
#
#             place = OrderedDict()
#             place['process_name'] = 'place'
#             place['base_frame'] = cframe2json(ao.base_frame)
#             place['target_pose'] = cframe2json(ao.shape_place_pose)
#             place['allowed_collision_obj_names'] = [] # support tables
#             for nghb_id in self.vertex_neighborhood(v):
#                 place['allowed_collision_obj_names'].extend(
#                     [obj_name(nghb_id, sub_id) \
#                         for sub_id in range(len(self.assembly_object(nghb_id).shape))])
#
#             grasp_data['processes'] = [pick, place]
#             e_data['grasps'] = grasp_data
#
#             data['sequenced_elements'].append(e_data)
#
#         with open(json_file_path, 'w') as outfile:
#             json.dump(data, outfile, indent=4)
#
#     def save_assembly_network_to_urdf(self, urdf_path):
#         pass
#
#     def save_to_assembly_planning_pkg(self, save_path, pkg_name, \
#         assembly_type="", model_type="", unit=""):
#         root_path = os.path.join(save_path, pkg_name)
#         if not os.path.isdir(root_path):
#             os.mkdir(root_path)
#
#         json_path = os.path.join(root_path, "json")
#         mesh_path = os.path.join(root_path, "meshes", "collision")
#         urdf_path = os.path.join(root_path, "urdf")
#         check_paths = [json_path, mesh_path, urdf_path]
#         for p in check_paths:
#             if not os.path.isdir(p):
#                 os.mkdir(p)
#
#         # generate obj files
#         self.save_assembly_object_to_objs(mesh_path)
#
#         # generate json
#         self.save_assembly_network_to_json(json_path, pkg_name, assembly_type, model_type, unit)
#
#         # genereate urdf
# #        self.save_assembly_network_to_urdf(urdf_path)

if __name__ == "__main__":
    pass
    # from compas.datastructures import Mesh
    # from compas_fab.assembly import Element

    # assembly = Assembly()
    # mesh = Mesh.from_polyhedron(4)

    # for i in range(2):
    #     element = Element.from_mesh(mesh)
    #     assembly.add_element(element)

    # print(assembly.summary())
