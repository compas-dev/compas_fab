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
from .utils import transform_cmesh, element_vert_key, \
    virtual_joint_key, extract_element_vert_id, obj_name, STATIC_OBSTACLE_PREFIX


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
        self._element_geometries = {} # object frame moved to world frame
        self._virtual_joint_geometries = {}
        self._static_obstacle_geometries = {}
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
        if isinstance(e_id, int):
            e_key = element_vert_key(e_id)
        elif isinstance(e_id, str) and extract_element_vert_id(e_id) is not None:
            e_key = e_id
        else:
            return None

        # assert(element_vert_key(e_id) in self._net.vertex)
        return self._net.get_vertex_attribute(e_key, 'element')

    @property
    def elements(self):
        e_dict = dict()
        for v_key in self._net.vertex.keys():
            # is_e = extract_element_vert_id(v_key)
            # if is_e is not None:
            e_dict[v_key] = self.get_element(v_key)
        return e_dict

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

    @property
    def static_obstacle_geometries(self):
        return self._static_obstacle_geometries

    @static_obstacle_geometries.setter
    def static_obstacle_geometries(self, cmesh_lists):
        # assert(isinstance(cmesh_lists, list))
        self._static_obstacle_geometries = {STATIC_OBSTACLE_PREFIX + '_' + str(id) : cl for id, cl in enumerate(cmesh_lists)}

    # --------------
    # exporters
    # --------------
    def save_element_geometries_to_objs(self, mesh_path):
        if not os.path.isdir(mesh_path):
            os.mkdir(mesh_path)
        for eg_key, eg_val in self._element_geometries.items():
            for sub_id, cm in enumerate(eg_val):
                cm.to_obj(os.path.join(mesh_path, obj_name(eg_key, sub_id)))

    def save_static_obstacles_geometries_to_objs(self, mesh_path):
        if not os.path.isdir(mesh_path):
            os.mkdir(mesh_path)
        for eg_key, eg_val in self.static_obstacle_geometries.items():
            for sub_id, cm in enumerate(eg_val):
                cm.to_obj(os.path.join(mesh_path, obj_name(eg_key, sub_id)))

    def save_assembly_to_json(self, json_path, pkg_name='', assembly_type='', model_type='', unit='', given_seq=None):
        if not os.path.isdir(json_path):
            os.mkdir(json_path)
        json_file_path = os.path.join(json_path, pkg_name + '.json')

        data = OrderedDict()
        data['pkg_name'] = pkg_name
        data['assembly_type'] = assembly_type
        data['model_type'] = model_type
        data['unit'] = unit
        data['with_given_sequence'] = bool(given_seq)
        # TODO: sanity check: vj geo + element geo = given seq

        data['element_number'] = self._num_of_elements
        data['sequenced_elements'] = OrderedDict()

        for e, order_id in zip(self.elements.values(), given_seq):
            e_data = OrderedDict()
            e_data['order_id'] = order_id
            e_data['object_id'] = e.key
            e_data['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            e_data['element_geometry_file_names'] = {sub_id : {'full_obj' : obj_name(e.key, sub_id)} \
                for sub_id in range(len(self._element_geometries[e.key]))}

            e_data['assembly_process'] = OrderedDict()
            pick = OrderedDict()
            pick['process_name'] = 'pick'
            pick['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            pick['object_target_pose'] = Transformation.from_frame(e.world_from_element_pick_pose).list
            pick['allowed_collision_obj_names'] = [] # support tables, neighbor elements?
            # old names: pick_contact_ngh_ids, pick_support_surface_file_names
            pick['grasp_from_approach_tf'] = e.grasp_from_approach_tf.list
            e_data['assembly_process']['pick'] = pick

            place = OrderedDict()
            place['process_name'] = 'place'
            place['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            place['object_target_pose'] = Transformation.from_frame(e.world_from_element_place_pose).list
            place['allowed_collision_obj_names'] = [] # support tables
            place['grasp_from_approach_tf'] = e.grasp_from_approach_tf.list
            e_data['assembly_process']['place'] = place
            # TODO: neighbor ACM
            # for nghb_id in self.vertex_neighborhood(v):
            #     place['allowed_collision_obj_names'].extend(
            #         [obj_name(nghb_id, sub_id) \
            #             for sub_id in range(len(self.assembly_object(nghb_id).shape))])

            grasp_data = OrderedDict()
            grasp_data['parent_link'] = 'object'
            grasp_data['ee_poses'] = [Transformation.from_frame(ee_p).list \
                for ee_p in e.obj_from_grasp_poses]

            e_data['grasps'] = grasp_data
            data['sequenced_elements'][e.key] = e_data

        data['static_obstacles'] = OrderedDict()
        for cl_key, cl in self.static_obstacle_geometries.items():
            data['static_obstacles'][cl_key] = OrderedDict()
            for sub_id in range(len(cl)):
                data['static_obstacles'][cl_key][sub_id] = OrderedDict()
                data['static_obstacles'][cl_key][sub_id]['full_obj'] = \
                obj_name(cl_key , sub_id)

        with open(json_file_path, 'w') as outfile:
            json.dump(data, outfile, indent=4)

    # def save_assembly_to_urdf(self, urdf_path):
    #     pass

    def save_to_assembly_planning_pkg(self, save_path, pkg_name, \
        assembly_type="", model_type="", unit="", given_seq=None):
        root_path = os.path.join(save_path, pkg_name)
        if not os.path.isdir(root_path):
            os.mkdir(root_path)

        json_path = os.path.join(root_path, "json")
        mesh_path = os.path.join(root_path, "meshes", "collision")
        urdf_path = os.path.join(root_path, "urdf")
        check_paths = [json_path, mesh_path, urdf_path]
        for p in check_paths:
            if not os.path.isdir(p):
                os.mkdir(p)

        # generate obj files
        self.save_element_geometries_to_objs(mesh_path)
        self.save_static_obstacles_geometries_to_objs(mesh_path)

        # generate json
        self.save_assembly_to_json(json_path, pkg_name, assembly_type, model_type, unit, given_seq)

        # TODO: genereate static collision objects urdf
        # self.generate_env_collision_objects_urdf(collision_objs, urdf_path)

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
