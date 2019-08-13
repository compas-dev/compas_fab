""" file IO util functions for parsing problem instance / saving results

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import json
from copy import copy, deepcopy

from conrob_pybullet import *
from compas_fab.assembly.datastructures import UnitGeometry, Grasp

from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation

def parse_transform(json_tf, scale=1.0, parse_matrix=False):
    if parse_matrix:
        tf = [json_tf[i*4:i*4 + 4] for i in range(4)]
        for i in range(3):
            tf[i][3] *= scale # translation entry
    else:
        tf = json_tf
        for i in range(3):
            tf[i*4 + 3] *= scale # translation entry
    return tf

def scale_mesh(mesh, scale=1.0):
    for key, v in mesh.vertex.items():
        for k in v.keys():
            mesh.vertex[key][k] *= scale
    return mesh


def extract_file_name(str_key):
    key_sep = str_key.split('.')
    return key_sep[0]


def warning_print(msg):
    print('\x1b[6;30;43m' + msg + '\x1b[0m')


def load_pick_and_place(instance_dir, instance_name, scale=1.0):
    with open(os.path.join(instance_dir, instance_name, 'json', instance_name + '.json'), 'r') as f:
        json_data = json.loads(f.read())

    obj_directory = os.path.join(instance_dir, instance_name, 'meshes', 'collision')

    brick_from_index = {}
    for e_id, json_element in json_data['sequenced_elements'].items():
        index = json_element['object_id']
        assert(e_id == index)
        # TODO: transform geometry based on json_element['parent_frame']

        ## parsing poses
        obj_from_ee_grasp_poses = [Frame.from_list(parse_transform(json_tf, scale=scale)) \
                                   for json_tf in json_element['grasps']['ee_poses']]

        pick_parent_frame = \
        Frame.from_list(parse_transform(json_element['assembly_process']['pick']['parent_frame'], scale=scale))
        world_from_obj_pick = \
        pick_parent_frame.transformed(
            Transformation.from_list(
            parse_transform(json_element['assembly_process']['pick']['object_target_pose'], scale=scale))
        )

        place_parent_frame = \
        Frame.from_list(parse_transform(json_element['assembly_process']['place']['parent_frame']))
        world_from_obj_place = \
        place_parent_frame.transformed(
            Transformation.from_list(
            parse_transform(json_element['assembly_process']['place']['object_target_pose'], scale=scale))
        )

        # TODO: pick and place might have different approach tfs
        ee_from_approach_tf = Transformation.from_list(
            parse_transform(json_element['assembly_process']['place']['grasp_from_approach_tf'], scale=scale))

        def multiply_frame_tf(frame, tf):
            tf_frame = Transformation.from_frame(frame)
            return Frame.from_transformation(\
                Transformation.concatenate(tf_frame, tf))

        obj_from_ee_grasps = [Grasp(index, grasp_id, \
                                    multiply_frame_tf(obj_from_ee_pose, ee_from_approach_tf),
                                    obj_from_ee_pose,
                                    multiply_frame_tf(obj_from_ee_pose, ee_from_approach_tf),
                                    # obj_from_ee_pose.transformed(ee_from_approach_tf),
                                     ) \
                              for grasp_id, obj_from_ee_pose in enumerate(obj_from_ee_grasp_poses)]

        ## parsing geometries
        element_meshes = []
        for sub_id, sub_dict in json_element['element_geometry_file_names'].items():
            if 'convex_decomp' in sub_dict and sub_dict['convex_decomp']:
                for part_obj_file in sub_dict['convex_decomp']:
                    sub_mesh = Mesh.from_obj(os.path.join(obj_directory, part_obj_file))
                    scale_mesh(sub_mesh, scale=scale)
                    element_meshes.append(sub_mesh)
            else:
                warning_print('warning: E#{} does not have convex decomp entries, use full body instead.'.format(index))
                full_obj_file = sub_dict['full_obj']
                full_mesh = Mesh.from_obj(os.path.join(obj_directory, full_obj_file))
                scale_mesh(full_mesh, scale=scale)
                element_meshes.append(full_mesh)

        brick_from_index[index] = UnitGeometry(name=index, mesh=element_meshes,
                                               initial_frame=world_from_obj_pick,
                                               goal_frame=world_from_obj_place,
                                               grasps=obj_from_ee_grasps)
        # pick_contact_ngh_ids are movable element contact partial orders
        # pick_support_surface_file_names are fixed element contact partial orders

    # static collision
    obstacle_from_name = {}
    for so_name, so_dict in json_data['static_obstacles'].items():
        for sub_id, so in so_dict.items():
            if 'convex_decomp' in so and so['convex_decomp']:
                for cvd_obj in so['convex_decomp']:
                    obj_name = extract_file_name(cvd_obj)
                    sub_mesh = Mesh.from_obj(os.path.join(obj_directory, cvd_obj))
                    scale_mesh(sub_mesh, scale=scale)
                    assert obj_name not in obstacle_from_name
                    obstacle_from_name[obj_name] = sub_mesh
            else:
                warning_print('warning: E#{} does not have convex decomp entries, use full body instead.'.format(index))
                full_obj_file = so['full_obj']
                full_mesh = Mesh.from_obj(os.path.join(obj_directory, full_obj_file))
                scale_mesh(full_mesh, scale=scale)
                obj_name = extract_file_name(full_obj_file)
                assert obj_name not in obstacle_from_name
                obstacle_from_name[obj_name] = full_mesh

    return brick_from_index, obstacle_from_name

